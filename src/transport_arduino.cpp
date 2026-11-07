/*
 * transport_arduino.cpp  (STM32 version for PlatformIO + Arduino framework)
 *
 * TELEM2 link to the Cube: USART1 on PA9 (TX) / PA10 (RX), 57600 baud.
 * Debug messages go to the USB cable (ST-LINK virtual COM port) at 115200,
 * so you can read them in PlatformIO's Serial Monitor.
 */
#include <Arduino.h>
#include <stdarg.h>
#include <stdio.h>
#include "transport.h"

static Uart TelemSerial(PA10, PA9);   /* (RX pin, TX pin) */

int transport_init(void)
{
    Serial.begin(115200);       /* USB debug output */
    TelemSerial.begin(57600);   /* Cube TELEM2 */
    return 0;
}

int transport_send(const uint8_t *buf, uint16_t len)
{
    return (int)TelemSerial.write(buf, len);
}

int transport_read(uint8_t *buf, int max)
{
    int n = 0;
    while (n < max && TelemSerial.available() > 0) {
        buf[n++] = (uint8_t)TelemSerial.read();
    }
    return n;
}

uint32_t transport_millis(void)
{
    return millis();
}

void transport_log(const char *fmt, ...)
{
    char line[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(line, sizeof(line), fmt, args);
    va_end(args);
    Serial.println(line);
}