#!/usr/bin/env python3
"""
test_protocol.py - Pytest for ESP32 USB protocol testing.

Usage:
    pytest utils/test_protocol.py -v
"""

from protocol_common import (
    ERR_LEN,
    ERR_TYPE,
    MSG_CMD_CONFIG,
    MSG_CMD_MOTOR,
    MSG_CMD_SERVO,
    MSG_HEARTBEAT,
    encode_config,
    encode_motor,
    encode_servo,
)


def test_heartbeat(serial_agent):
    seq = serial_agent.send_frame(MSG_HEARTBEAT, b"")
    assert serial_agent.wait_for_ack(seq)


def test_motor_command(serial_agent):
    payload = encode_motor(512, -512)
    seq = serial_agent.send_frame(MSG_CMD_MOTOR, payload)
    assert serial_agent.wait_for_ack(seq)


def test_servo_command(serial_agent):
    payload = encode_servo(0, 1500)
    seq = serial_agent.send_frame(MSG_CMD_SERVO, payload)
    assert serial_agent.wait_for_ack(seq)


def test_config_command(serial_agent):
    payload = encode_config(1, 100)
    seq = serial_agent.send_frame(MSG_CMD_CONFIG, payload)
    assert serial_agent.wait_for_ack(seq)


def test_invalid_command(serial_agent):
    seq = serial_agent.send_frame(0xFF, b"test")
    payload = serial_agent.wait_for_nack(seq)
    assert payload == bytes([seq, ERR_TYPE])


def test_invalid_motor_length(serial_agent):
    seq = serial_agent.send_frame(MSG_CMD_MOTOR, b"\x00")
    payload = serial_agent.wait_for_nack(seq)
    assert payload == bytes([seq, ERR_LEN])
