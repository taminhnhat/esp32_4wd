#pragma once

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif


void usb_bridge_init(void);
void usb_bridge_task(void *pvParameters);
void usb_bridge_write(const char *data, size_t len);

#ifdef __cplusplus
}
#endif
