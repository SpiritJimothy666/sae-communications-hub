/*
 * mission_resume.c
 *
 * C version of resume_mission.py.
 *
 *   1. Wait for the plane to arm (flight 1), then disarm (landed at the DLZ)
 *   2. Wait for the pickup (for now: a fixed delay)
 *   3. MISSION_SET_CURRENT -> ARM -> MISSION_START, from RESUME_SEQ
 *      (PX4 rejects the DO_SET_MISSION_CURRENT command as unsupported)
 *
 * Written as a state machine: mission_resume_update() is called over and
 * over; each call does a little work and returns right away.
 * If a step fails, it stops (ST_FAILED) instead of arming the plane.
 */
#include "mission_resume.h"
#include "transport.h"

#include <common/mavlink.h>   /* MAVLink C library (c_library_v2) */

/* ---- Settings ----------------------------------------------------------- */
#define MY_SYSID            1     /* same system as the Cube (from Grant)   */
#define MY_COMPID           191   /* companion computer component ID         */
#define TARGET_SYSID        1     /* the Cube / PX4                          */
#define TARGET_COMPID       1     /* the autopilot component                 */

#define RESUME_SEQ          3     /* mission item to restart from            */
#define PICKUP_WAIT_MS      5000  /* TODO: replace with the real SECURED signal */

#define HEARTBEAT_PERIOD_MS 1000  /* send our heartbeat once a second        */
#define ACK_TIMEOUT_MS      3000  /* wait this long for a COMMAND_ACK        */
#define MAX_RETRIES         3     /* resend a command this many times        */

/* ---- State machine ------------------------------------------------------ */
typedef enum {
    ST_WAIT_ARMED,      /* waiting for flight 1 to start          */
    ST_WAIT_DISARMED,   /* flying; waiting for landing + disarm   */
    ST_WAIT_PICKUP,     /* on the ground; pickup happening        */
    ST_SET_CURRENT,     /* sending MISSION_SET_CURRENT            */
    ST_ARM,             /* sending ARM                            */
    ST_START,           /* sending MISSION_START                  */
    ST_DONE,            /* flight 2 started                       */
    ST_FAILED           /* a step failed; stop here               */
} state_t;

static state_t  state;
static uint32_t state_start_ms;
static uint32_t last_heartbeat_ms;

static int autopilot_armed;        /* armed flag from the Cube's heartbeat */
static int connected;              /* have we heard from the Cube yet?     */

static int      cmd_pending;       /* waiting for an ACK?                  */
static uint16_t pending_cmd;       /* which command we're waiting on       */
static int      ack_result;        /* -1 = no ACK yet, else MAV_RESULT     */
static uint32_t cmd_sent_ms;
static int      retries;

static int      mission_current = -1;  /* current item PX4 reports */
static uint32_t set_current_sent_ms;

static void set_state(state_t s)
{
    state = s;
    state_start_ms = transport_millis();
    if (s == ST_FAILED) {
        transport_log("Stopped: a step failed, not continuing");
    }
}

/* ---- Sending ------------------------------------------------------------ */
static void send_message(mavlink_message_t *msg)
{
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    uint16_t len = mavlink_msg_to_send_buffer(buf, msg);
    transport_send(buf, len);
}

static void send_heartbeat(void)
{
    mavlink_message_t msg;
    mavlink_msg_heartbeat_pack(MY_SYSID, MY_COMPID, &msg,
                               MAV_TYPE_ONBOARD_CONTROLLER,
                               MAV_AUTOPILOT_INVALID,
                               0, 0, MAV_STATE_ACTIVE);
    send_message(&msg);
}

static void send_command(uint16_t cmd, float p1, float p2)
{
    mavlink_message_t msg;
    mavlink_msg_command_long_pack(MY_SYSID, MY_COMPID, &msg,
                                  TARGET_SYSID, TARGET_COMPID,
                                  cmd, 0,
                                  p1, p2, 0, 0, 0, 0, 0);
    send_message(&msg);
    pending_cmd = cmd;
    ack_result  = -1;
    cmd_sent_ms = transport_millis();
}

/* Jump to a mission item. No ACK; PX4 confirms through MISSION_CURRENT. */
static void send_set_current(uint16_t seq)
{
    mavlink_message_t msg;
    mavlink_msg_mission_set_current_pack(MY_SYSID, MY_COMPID, &msg,
                                         TARGET_SYSID, TARGET_COMPID, seq);
    send_message(&msg);
    set_current_sent_ms = transport_millis();
}

/* Send a command and wait for its ACK. 1 = accepted, -1 = failed, 0 = waiting */
static int command_step(const char *name, uint16_t cmd, float p1, float p2)
{
    if (!cmd_pending) {
        retries = 0;
        send_command(cmd, p1, p2);
        cmd_pending = 1;
        return 0;
    }

    if (ack_result >= 0) {
        if (ack_result == MAV_RESULT_IN_PROGRESS) {
            ack_result = -1;
            return 0;
        }
        cmd_pending = 0;
        if (ack_result == MAV_RESULT_ACCEPTED) {
            transport_log("%s: accepted", name);
            return 1;
        }
        transport_log("%s: rejected (result %d)", name, ack_result);
        return -1;
    }

    if (transport_millis() - cmd_sent_ms > ACK_TIMEOUT_MS) {
        if (++retries > MAX_RETRIES) {
            cmd_pending = 0;
            transport_log("%s: no reply", name);
            return -1;
        }
        transport_log("%s: no reply, retrying", name);
        send_command(cmd, p1, p2);
    }
    return 0;
}

/* ---- Receiving ---------------------------------------------------------- */
static void handle_message(const mavlink_message_t *msg)
{
    switch (msg->msgid) {

    case MAVLINK_MSG_ID_HEARTBEAT:
        if (msg->sysid == TARGET_SYSID && msg->compid == TARGET_COMPID) {
            mavlink_heartbeat_t hb;
            mavlink_msg_heartbeat_decode(msg, &hb);
            autopilot_armed = (hb.base_mode & MAV_MODE_FLAG_SAFETY_ARMED) != 0;
            if (!connected) {
                connected = 1;
                transport_log("Connected to PX4");
            }
        }
        break;

    case MAVLINK_MSG_ID_MISSION_CURRENT:
        if (msg->sysid == TARGET_SYSID && msg->compid == TARGET_COMPID) {
            mission_current = mavlink_msg_mission_current_get_seq(msg);
        }
        break;

    case MAVLINK_MSG_ID_COMMAND_ACK: {
        mavlink_command_ack_t ack;
        mavlink_msg_command_ack_decode(msg, &ack);
        if (cmd_pending && ack.command == pending_cmd) {
            ack_result = ack.result;
        }
        break;
    }

    default:
        break;
    }
}

/* ---- Public functions --------------------------------------------------- */
void mission_resume_init(void)
{
    autopilot_armed   = 0;
    connected         = 0;
    cmd_pending       = 0;
    ack_result        = -1;
    last_heartbeat_ms = 0;
    set_state(ST_WAIT_ARMED);
}

void mission_resume_update(void)
{
    /* 1. Feed every received byte into the MAVLink parser */
    static uint8_t buf[256];
    int n;
    while ((n = transport_read(buf, sizeof(buf))) > 0) {
        for (int i = 0; i < n; i++) {
            mavlink_message_t msg;
            mavlink_status_t status;
            if (mavlink_parse_char(MAVLINK_COMM_0, buf[i], &msg, &status)) {
                handle_message(&msg);
            }
        }
    }

    /* 2. Heartbeat once a second */
    uint32_t now = transport_millis();
    if (now - last_heartbeat_ms >= HEARTBEAT_PERIOD_MS) {
        send_heartbeat();
        last_heartbeat_ms = now;
    }

    if (!connected) {
        return;
    }

    /* 3. Mission logic */
    int r;
    switch (state) {

    case ST_WAIT_ARMED:
        if (autopilot_armed) {
            transport_log("Plane armed, flight 1 running");
            set_state(ST_WAIT_DISARMED);
        }
        break;

    case ST_WAIT_DISARMED:
        if (!autopilot_armed) {
            transport_log("Plane landed and disarmed");
            transport_log("Waiting %d ms for pickup...", PICKUP_WAIT_MS);
            set_state(ST_WAIT_PICKUP);
        }
        break;

    case ST_WAIT_PICKUP:
        if (now - state_start_ms >= PICKUP_WAIT_MS) {
            set_current_sent_ms = now - 1000;   /* send right away */
            set_state(ST_SET_CURRENT);
        }
        break;

    case ST_SET_CURRENT:
        if (mission_current == RESUME_SEQ) {
            transport_log("Set current item: now at item %d", RESUME_SEQ);
            set_state(ST_ARM);
        } else if (now - state_start_ms > 5000) {
            transport_log("Set current item: PX4 still at item %d", mission_current);
            set_state(ST_FAILED);
        } else if (now - set_current_sent_ms >= 500) {
            send_set_current(RESUME_SEQ);   /* resend every 0.5 s until confirmed */
        }
        break;

    case ST_ARM:
        r = command_step("Arm", MAV_CMD_COMPONENT_ARM_DISARM, 1, 0);
        if (r == 1)      set_state(ST_START);
        else if (r < 0)  set_state(ST_FAILED);
        break;

    case ST_START:
        r = command_step("Start mission", MAV_CMD_MISSION_START, RESUME_SEQ, 0);
        if (r == 1) {
            transport_log("Done, flight 2 should be starting");
            set_state(ST_DONE);
        } else if (r < 0) {
            set_state(ST_FAILED);
        }
        break;

    case ST_DONE:
    case ST_FAILED:
        break;
    }
}
