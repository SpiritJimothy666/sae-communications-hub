/*
 * main.cpp  (PlatformIO entry point)
 * Runs the mission-resume state machine forever.
 * The green LED blinks once a second so you can see the code is running.
 */
#include <Arduino.h>
#include "transport.h"
#include "mission_resume.h"

static uint32_t last_blink = 0;

void setup()
{
    pinMode(LED_BUILTIN, OUTPUT);
    transport_init();
    delay(1000);
    transport_log("STM32 mission-resume starting (sysid 1, compid 191)");
    mission_resume_init();
}

void loop()
{
    mission_resume_update();

    if (millis() - last_blink >= 500) {
        digitalWrite(LED_BUILTIN, !digitalRead(LED_BUILTIN));
        last_blink = millis();
    }
}
