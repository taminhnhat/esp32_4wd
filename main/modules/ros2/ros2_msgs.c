#include "ros2_msgs.h"

#include <string.h>
#include "board_led.h"
#include "esp_log.h"

static const char *TAG = "ros2_msgs";

#define ROS2_MSG_SOF 0xAA
#define ROS2_MSG_ESCAPE 0x1B
#define ROS2_MSG_ESCAPE_XOR 0x20

#define ROS2_MSG_HEARTBEAT 0x00
#define ROS2_MSG_CMD_MOTOR 0x01
#define ROS2_MSG_CMD_SERVO 0x02
#define ROS2_MSG_CMD_CONFIG 0x10
#define ROS2_MSG_ACK 0x7E
#define ROS2_MSG_NACK 0x7F

#define ROS2_MSG_ERR_CRC 0x01
#define ROS2_MSG_ERR_LEN 0x02
#define ROS2_MSG_ERR_TYPE 0x03

static uint16_t ros2_msgs_crc16(const uint8_t *data, size_t len)
{
    uint16_t crc = 0xFFFF;

    for (size_t i = 0; i < len; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int bit = 0; bit < 8; bit++)
        {
            crc = (crc & 0x8000) ? ((crc << 1) ^ 0x1021) : (crc << 1);
            crc &= 0xFFFF;
        }
    }

    return crc;
}

static size_t ros2_msgs_unstuff(const uint8_t *stuffed, size_t len, uint8_t *unstuffed, size_t unstuffed_size)
{
    size_t out_len = 0;

    for (size_t i = 0; i < len; i++)
    {
        uint8_t value = stuffed[i];

        if (value == ROS2_MSG_ESCAPE)
        {
            i++;
            if (i >= len)
            {
                break;
            }
            value = stuffed[i] ^ ROS2_MSG_ESCAPE_XOR;
        }

        if (out_len >= unstuffed_size)
        {
            break;
        }
        unstuffed[out_len++] = value;
    }

    return out_len;
}

static size_t ros2_msgs_stuff(const uint8_t *data, size_t len, uint8_t *stuffed, size_t stuffed_size)
{
    size_t out_len = 0;

    for (size_t i = 0; i < len; i++)
    {
        if (data[i] == ROS2_MSG_SOF || data[i] == ROS2_MSG_ESCAPE)
        {
            if (out_len + 2 > stuffed_size)
            {
                break;
            }
            stuffed[out_len++] = ROS2_MSG_ESCAPE;
            stuffed[out_len++] = data[i] ^ ROS2_MSG_ESCAPE_XOR;
        }
        else
        {
            if (out_len + 1 > stuffed_size)
            {
                break;
            }
            stuffed[out_len++] = data[i];
        }
    }

    return out_len;
}

static void ros2_msgs_write(ros2_msgs_t *msgs, const uint8_t *data, size_t len)
{
    if (msgs->write_fn != NULL)
    {
        msgs->write_fn(data, len, msgs->write_ctx);
    }
}

void ros2_msgs_send_frame(ros2_msgs_t *msgs, uint8_t msg_type, uint8_t seq, const uint8_t *payload, size_t payload_len)
{
    uint8_t stuffed[512];
    const size_t stuffed_len = ros2_msgs_stuff(payload, payload_len, stuffed, sizeof(stuffed));
    if (payload_len > 0 && stuffed_len == 0)
    {
        ESP_LOGW(TAG, "Failed to stuff payload type=0x%02X seq=%u", msg_type, seq);
        return;
    }

    uint8_t frame[1 + 4 + sizeof(stuffed) + 2];
    frame[0] = ROS2_MSG_SOF;
    frame[1] = msg_type;
    frame[2] = seq;
    frame[3] = stuffed_len & 0xFF;
    frame[4] = (stuffed_len >> 8) & 0xFF;
    memcpy(frame + 5, stuffed, stuffed_len);

    const uint16_t crc = ros2_msgs_crc16(frame + 1, 4 + stuffed_len);
    frame[5 + stuffed_len] = crc & 0xFF;
    frame[6 + stuffed_len] = (crc >> 8) & 0xFF;

    ros2_msgs_write(msgs, frame, 7 + stuffed_len);
}

static void ros2_msgs_send_ack(ros2_msgs_t *msgs, uint8_t seq)
{
    ros2_msgs_send_frame(msgs, ROS2_MSG_ACK, seq, &seq, 1);
}

static void ros2_msgs_send_nack(ros2_msgs_t *msgs, uint8_t seq, uint8_t err)
{
    uint8_t payload[2] = {seq, err};
    ros2_msgs_send_frame(msgs, ROS2_MSG_NACK, seq, payload, sizeof(payload));
}

static void ros2_msgs_handle_message(ros2_msgs_t *msgs, uint8_t msg_type, uint8_t seq, const uint8_t *payload, size_t len)
{
    switch (msg_type)
    {
    case ROS2_MSG_HEARTBEAT:
        led_set(8, 0, 8);
        ESP_LOGI(TAG, "Received HEARTBEAT seq=%u", seq);
        ros2_msgs_send_ack(msgs, seq);
        break;

    case ROS2_MSG_CMD_MOTOR:
        led_set(8, 8, 0);
        if (len == 4)
        {
            const int16_t left = (int16_t)(payload[0] | (payload[1] << 8));
            const int16_t right = (int16_t)(payload[2] | (payload[3] << 8));
            ESP_LOGI(TAG, "Received CMD_MOTOR seq=%u left=%d right=%d", seq, left, right);
            ros2_msgs_send_ack(msgs, seq);
        }
        else
        {
            ros2_msgs_send_nack(msgs, seq, ROS2_MSG_ERR_LEN);
        }
        break;

    case ROS2_MSG_CMD_SERVO:
        led_set(0, 0, 16);
        if (len == 3)
        {
            const uint8_t channel = payload[0];
            const uint16_t pulse = payload[1] | (payload[2] << 8);
            ESP_LOGI(TAG, "Received CMD_SERVO seq=%u channel=%u pulse=%u", seq, channel, pulse);
            ros2_msgs_send_ack(msgs, seq);
        }
        else
        {
            ros2_msgs_send_nack(msgs, seq, ROS2_MSG_ERR_LEN);
        }
        break;

    case ROS2_MSG_CMD_CONFIG:
        led_set(0, 8, 8);
        if (len == 5)
        {
            const uint8_t key = payload[0];
            const uint32_t raw_value = (uint32_t)payload[1] |
                                       ((uint32_t)payload[2] << 8) |
                                       ((uint32_t)payload[3] << 16) |
                                       ((uint32_t)payload[4] << 24);
            const int32_t value = (int32_t)raw_value;
            ESP_LOGI(TAG, "Received CMD_CONFIG seq=%u key=%u value=%d", seq, key, value);
            ros2_msgs_send_ack(msgs, seq);
        }
        else
        {
            ros2_msgs_send_nack(msgs, seq, ROS2_MSG_ERR_LEN);
        }
        break;

    default:
        led_set(16, 0, 0);
        ESP_LOGW(TAG, "Unknown message type 0x%02X seq=%u", msg_type, seq);
        ros2_msgs_send_nack(msgs, seq, ROS2_MSG_ERR_TYPE);
        break;
    }
}

void ros2_msgs_init(ros2_msgs_t *msgs, ros2_msgs_write_fn_t write_fn, void *write_ctx)
{
    memset(msgs, 0, sizeof(*msgs));
    msgs->write_fn = write_fn;
    msgs->write_ctx = write_ctx;
}

void ros2_msgs_on_rx(ros2_msgs_t *msgs, const uint8_t *data, size_t len)
{
    if (len == 0)
    {
        return;
    }
    ESP_LOGI(TAG, "Received %u bytes", len);

    if (msgs->parse_len + len > sizeof(msgs->parse_buf))
    {
        msgs->parse_len = 0;
        ESP_LOGW(TAG, "Parse buffer overflow, resetting");
    }

    if (len > sizeof(msgs->parse_buf))
    {
        data += len - sizeof(msgs->parse_buf);
        len = sizeof(msgs->parse_buf);
    }

    memcpy(msgs->parse_buf + msgs->parse_len, data, len);
    msgs->parse_len += len;

    size_t offset = 0;
    while (offset + 7 <= msgs->parse_len)
    {
        if (msgs->parse_buf[offset] != ROS2_MSG_SOF)
        {
            offset++;
            continue;
        }

        const uint8_t msg_type = msgs->parse_buf[offset + 1];
        const uint8_t seq = msgs->parse_buf[offset + 2];
        const uint16_t length = msgs->parse_buf[offset + 3] | (msgs->parse_buf[offset + 4] << 8);
        const size_t frame_len = length + 7;

        if (length > 512)
        {
            ESP_LOGW(TAG, "Invalid frame length %u", length);
            ros2_msgs_send_nack(msgs, seq, ROS2_MSG_ERR_LEN);
            offset++;
            continue;
        }

        if (offset + frame_len > msgs->parse_len)
        {
            ESP_LOGI(TAG, "Incomplete frame, waiting for more data");
            break;
        }

        const uint8_t *stuffed_payload = msgs->parse_buf + offset + 5;
        const uint16_t crc_received = msgs->parse_buf[offset + 5 + length] | (msgs->parse_buf[offset + 6 + length] << 8);
        const uint16_t crc_computed = ros2_msgs_crc16(msgs->parse_buf + offset + 1, 4 + length);

        if (crc_computed != crc_received)
        {
            ESP_LOGW(TAG, "CRC mismatch seq=%u computed=0x%04X received=0x%04X", seq, crc_computed, crc_received);
            ros2_msgs_send_nack(msgs, seq, ROS2_MSG_ERR_CRC);
            offset += frame_len;
            continue;
        }

        uint8_t payload[256];
        const size_t payload_len = ros2_msgs_unstuff(stuffed_payload, length, payload, sizeof(payload));
        ros2_msgs_handle_message(msgs, msg_type, seq, payload, payload_len);
        offset += frame_len;
    }

    if (offset > 0)
    {
        memmove(msgs->parse_buf, msgs->parse_buf + offset, msgs->parse_len - offset);
        msgs->parse_len -= offset;
    }
}
