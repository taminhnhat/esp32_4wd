#pragma once

#include <stddef.h>
#include <stdint.h>
#include "usb_bridge.h"

#ifdef __cplusplus
extern "C"
{
#endif

    /**
     * ROS2 messages transported over custom serial protocol.
     *
     * Frame format:
     *   | SOF | Type | Seq | Length | Payload | CRC16 |
     *
     * Field definitions:
     *   SOF     : 1 byte  (always 0xAA)
     *   Type    : 1 byte  Message type ID
     *   Seq     : 1 byte  Sequence counter
     *   Length  : 2 bytes Payload length in bytes (little-endian)
     *   Payload : Variable length, byte-stuffed for frame transparency
     *   CRC16   : 2 bytes CRC-16 checksum (little-endian)
     *
     * CRC coverage:
     *   Type + Seq + Length + Payload
     *   (SOF is excluded)
     */

    typedef size_t (*ros2_msgs_write_fn_t)(void *ctx, uint8_t *data, size_t len);
    typedef size_t (*ros2_msgs_read_fn_t)(void *ctx, uint8_t *data, size_t len);

    typedef struct
    {
        uint8_t parse_buf[1024];
        size_t parse_len;
        uint8_t tx_seq;
        uint8_t rx_seq;
        ros2_msgs_write_fn_t write;
        ros2_msgs_read_fn_t read;
    } ros2_msgs_ctx_t;

    void ros2_msgs_init(ros2_msgs_ctx_t *msgs);
    // void ros2_msgs_on_rx(ros2_msgs_ctx_t *msgs, const uint8_t *data, size_t len);
    void ros2_msgs_send_frame(ros2_msgs_ctx_t *msgs, uint8_t msg_type, uint8_t seq, const uint8_t *payload, size_t payload_len);
    void ros2_msgs_send_telemetry(ros2_msgs_ctx_t *msgs, uint8_t seq);

#ifdef __cplusplus
}
#endif
