#pragma once

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C"
{
#endif
    typedef void (*usb_bridge_cb_t)(void *ctx);

    void usb_bridge_init(void *ctx);
    size_t usb_bridge_write_bytes(void *ctx, uint8_t *data, size_t len);
    size_t usb_bridge_read_bytes(void *ctx, uint8_t *data, size_t len);
    void usb_bridge_set_callback(void *ctx, usb_bridge_cb_t cb);

#ifdef __cplusplus
}
#endif
