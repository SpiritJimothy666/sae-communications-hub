#ifndef TRANSPORT_H
#define TRANSPORT_H
#include <stdint.h>
#ifdef __cplusplus
extern "C" {
#endif
int transport_init(void);
int transport_send(const uint8_t *buf, uint16_t len);
int transport_read(uint8_t *buf, int max);
uint32_t transport_millis(void);
void transport_log(const char *fmt, ...);
#ifdef __cplusplus
}
#endif
#endif
