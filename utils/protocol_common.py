#!/usr/bin/env python3
"""Shared helpers for ESP32 USB protocol pytest tests."""

import queue
import struct
import threading
import time
from pathlib import Path

import pytest
import serial

try:
    import yaml
except ImportError:
    yaml = None


SOF = 0xAA
ESCAPE = 0x1B
ESCAPE_XOR = 0x20

MSG_HEARTBEAT = 0x00
MSG_CMD_MOTOR = 0x01
MSG_CMD_SERVO = 0x02
MSG_CMD_CONFIG = 0x10
MSG_DATA_IMU = 0x20
MSG_DATA_ENC = 0x21
MSG_ACK = 0x7E
MSG_NACK = 0x7F

TYPE_NAMES = {
    MSG_HEARTBEAT: "HEARTBEAT",
    MSG_CMD_MOTOR: "CMD_MOTOR",
    MSG_CMD_SERVO: "CMD_SERVO",
    MSG_CMD_CONFIG: "CMD_CONFIG",
    MSG_DATA_IMU: "DATA_IMU",
    MSG_DATA_ENC: "DATA_ENC",
    MSG_ACK: "ACK",
    MSG_NACK: "NACK",
}

ERR_OK = 0x00
ERR_CRC = 0x01
ERR_LEN = 0x02
ERR_TYPE = 0x03

ERR_NAMES = {
    ERR_OK: "OK",
    ERR_CRC: "ERR_CRC",
    ERR_LEN: "ERR_LEN",
    ERR_TYPE: "ERR_TYPE",
}

CONFIG_PATH = Path(__file__).with_name("test_config.yaml")


def load_test_config() -> dict:
    if yaml is None:
        return {}
    try:
        with CONFIG_PATH.open("r", encoding="utf-8") as f:
            return yaml.safe_load(f) or {}
    except FileNotFoundError:
        return {}


def crc16(data: bytes) -> int:
    crc = 0xFFFF
    for b in data:
        crc ^= b << 8
        for _ in range(8):
            crc = ((crc << 1) ^ 0x1021) if (crc & 0x8000) else (crc << 1)
            crc &= 0xFFFF
    return crc


def stuff(payload: bytes) -> bytes:
    out = bytearray()
    for b in payload:
        if b == SOF or b == ESCAPE:
            out.append(ESCAPE)
            out.append(b ^ ESCAPE_XOR)
        else:
            out.append(b)
    return bytes(out)


def unstuff(payload: bytes) -> bytes:
    out = bytearray()
    i = 0
    while i < len(payload):
        b = payload[i]
        if b == ESCAPE and i + 1 < len(payload):
            i += 1
            out.append(payload[i] ^ ESCAPE_XOR)
        else:
            out.append(b)
        i += 1
    return bytes(out)


def build_frame(msg_type: int, seq: int, payload: bytes) -> bytes:
    stuffed = stuff(payload)
    header = struct.pack("<BBH", msg_type, seq, len(stuffed))
    crc_val = crc16(header + stuffed)
    return bytes([SOF]) + header + stuffed + struct.pack("<H", crc_val)


def encode_motor(left_pwm: int, right_pwm: int) -> bytes:
    return struct.pack("<hh", left_pwm, right_pwm)


def encode_servo(channel: int, pulse_us: int) -> bytes:
    return struct.pack("<BH", channel, pulse_us)


def encode_config(key: int, value: int) -> bytes:
    return struct.pack("<Bi", key, value)


class FrameParser:
    """Stream-oriented parser: feed raw bytes, get complete frames back."""

    def __init__(self):
        self._buf = bytearray()

    def feed(self, data: bytes):
        self._buf.extend(data)
        return self._extract_frames()

    def _extract_frames(self):
        frames = []
        while True:
            try:
                start = self._buf.index(SOF)
            except ValueError:
                self._buf.clear()
                break

            if start > 0:
                self._buf = self._buf[start:]

            if len(self._buf) < 7:
                break

            msg_type = self._buf[1]
            seq = self._buf[2]
            length = struct.unpack_from("<H", self._buf, 3)[0]
            total = 1 + 1 + 1 + 2 + length + 2

            if len(self._buf) < total:
                break

            raw_payload = bytes(self._buf[5 : 5 + length])
            raw_crc = struct.unpack_from("<H", self._buf, 5 + length)[0]
            crc_data = bytes(self._buf[1 : 5 + length])
            computed = crc16(crc_data)

            self._buf = self._buf[total:]

            if computed != raw_crc:
                print(f"  [CRC FAIL] expected 0x{computed:04X} got 0x{raw_crc:04X}")
                continue

            payload = unstuff(raw_payload)
            frames.append((msg_type, seq, payload))

        return frames


class SerialAgent:
    def __init__(self, port: str, baudrate: int = 115200):
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.parser = FrameParser()
        self.rx_queue = queue.Queue()
        self._stop = threading.Event()
        self._seq = 0
        self._seq_lock = threading.Lock()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.01)
            print(f"[SERIAL] Connected to {self.port}")
        except serial.SerialException as e:
            raise pytest.skip(f"Cannot open serial port {self.port}: {e}")

    def _rx_loop(self):
        while not self._stop.is_set():
            try:
                read_size = self.ser.in_waiting or 1
                data = self.ser.read(read_size)
                if data:
                    frames = self.parser.feed(data)
                    for frame in frames:
                        self.rx_queue.put(frame)
            except Exception as e:
                if not self._stop.is_set():
                    print(f"[SERIAL] RX error: {e}")
                break

    def start_rx(self):
        rx_thread = threading.Thread(target=self._rx_loop, daemon=True)
        rx_thread.start()

    def send_frame(self, msg_type: int, payload: bytes) -> int:
        with self._seq_lock:
            seq = self._seq
            self._seq = (self._seq + 1) & 0xFF

        frame = build_frame(msg_type, seq, payload)
        self.ser.write(frame)
        name = TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")
        print(f"  [TX] {name} seq={seq} payload={payload.hex()}")
        return seq

    def wait_for_ack(self, expected_seq: int, timeout=1.0) -> bool:
        return self.wait_for_response(MSG_ACK, expected_seq, timeout) is not None

    def wait_for_nack(self, expected_seq: int, timeout=1.0):
        return self.wait_for_response(MSG_NACK, expected_seq, timeout)

    def wait_for_response(self, expected_type: int, expected_seq: int, timeout=1.0):
        start = time.monotonic()
        while time.monotonic() - start < timeout:
            remaining = max(0.0, timeout - (time.monotonic() - start))
            try:
                msg_type, seq, payload = self.rx_queue.get(timeout=min(0.1, remaining))
            except queue.Empty:
                continue

            if msg_type == expected_type and seq == expected_seq:
                name = TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")
                print(f"  [RX] {name} seq={seq} payload={payload.hex()}")
                return payload

            name = TYPE_NAMES.get(msg_type, f"0x{msg_type:02X}")
            print(f"  [RX] Unexpected {name} seq={seq} payload={payload.hex()}")

        name = TYPE_NAMES.get(expected_type, f"0x{expected_type:02X}")
        print(f"  [TIMEOUT] No {name} for seq={expected_seq}")
        return None

    def drain_rx(self):
        while True:
            try:
                self.rx_queue.get_nowait()
            except queue.Empty:
                return

    def close(self):
        self._stop.set()
        if self.ser:
            self.ser.close()


@pytest.fixture(scope="session")
def test_config():
    return load_test_config()


@pytest.fixture(scope="session")
def serial_agent(test_config):
    port = test_config.get("port")
    if not port:
        pytest.skip(f"No serial port configured in {CONFIG_PATH}")

    baudrate = test_config.get("baudrate", 115200)
    agent = SerialAgent(port, baudrate)
    agent.connect()
    agent.start_rx()
    time.sleep(0.2)
    yield agent
    agent.close()
