#pragma once

#include <stddef.h>
#include <stdint.h>

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

    typedef void (*ros2_msgs_write_fn_t)(const uint8_t *data, size_t len, void *ctx);

    typedef struct
    {
        uint8_t parse_buf[1024];
        size_t parse_len;
        ros2_msgs_write_fn_t write_fn;
        void *write_ctx;
    } ros2_msgs_t;

    void ros2_msgs_init(ros2_msgs_t *msgs, ros2_msgs_write_fn_t write_fn, void *write_ctx);
    void ros2_msgs_on_rx(ros2_msgs_t *msgs, const uint8_t *data, size_t len);
    void ros2_msgs_send_frame(ros2_msgs_t *msgs, uint8_t msg_type, uint8_t seq, const uint8_t *payload, size_t payload_len);

#ifdef __cplusplus
}
#endif
