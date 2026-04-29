#!/usr/bin/env python3
"""
test_perf.py - Basic ESP32 USB protocol latency/performance smoke test.

Optional keys in utils/test_config.yaml:
    perf_iterations: 50
    perf_warmup_iterations: 5
    perf_ack_timeout: 0.5
    perf_max_avg_ack_ms: 50
    perf_max_p95_ack_ms: 200
"""

import statistics
import time

from protocol_common import MSG_HEARTBEAT


def percentile(values, percent):
    if not values:
        return 0.0
    ordered = sorted(values)
    index = round((len(ordered) - 1) * percent / 100)
    return ordered[index]


def test_heartbeat_ack_latency(serial_agent, test_config):
    iterations = int(test_config.get("perf_iterations", 50))
    warmup_iterations = int(test_config.get("perf_warmup_iterations", 5))
    ack_timeout = float(test_config.get("perf_ack_timeout", 0.5))
    max_avg_ms = float(test_config.get("perf_max_avg_ack_ms", 50))
    max_p95_ms = float(test_config.get("perf_max_p95_ack_ms", 200))

    assert iterations > 0
    serial_agent.drain_rx()

    for _ in range(warmup_iterations):
        seq = serial_agent.send_frame(MSG_HEARTBEAT, b"")
        assert serial_agent.wait_for_ack(seq, timeout=ack_timeout)

    latencies_ms = []
    for _ in range(iterations):
        start = time.perf_counter()
        seq = serial_agent.send_frame(MSG_HEARTBEAT, b"")
        assert serial_agent.wait_for_ack(seq, timeout=ack_timeout)
        latencies_ms.append((time.perf_counter() - start) * 1000)

    avg_ms = statistics.fmean(latencies_ms)
    p95_ms = percentile(latencies_ms, 95)
    worst_ms = max(latencies_ms)
    hz = 1000 / avg_ms if avg_ms else float("inf")

    print(
        f"[PERF] heartbeat ACK: count={iterations} "
        f"avg={avg_ms:.2f}ms p95={p95_ms:.2f}ms worst={worst_ms:.2f}ms "
        f"rate={hz:.1f}Hz"
    )

    assert avg_ms <= max_avg_ms
    assert p95_ms <= max_p95_ms
