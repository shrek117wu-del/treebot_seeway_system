#!/usr/bin/env python3
"""
Unit tests for the LZ_OMNI chassis driver (protocol layer only – no hardware required).

Tests cover:
  - XOR checksum computation
  - UART frame construction  (uart_driver.build_motion_frame)
  - CAN payload construction (can_driver.build_can_motion_data)
  - Value clamping at protocol limits
"""

import struct
import sys
import os
import unittest

# Allow importing the package directly from the source tree
sys.path.insert(
    0,
    os.path.join(os.path.dirname(__file__), '..'),
)

from lz_omni_chassis_driver.uart_driver import (
    FRAME_HEADER0,
    FRAME_HEADER1,
    CMD_MOTION,
    build_motion_frame,
    compute_xor,
)
from lz_omni_chassis_driver.can_driver import (
    build_can_motion_data,
    compute_xor as can_compute_xor,
)


class TestComputeXor(unittest.TestCase):
    """Tests for the XOR checksum helper."""

    def test_empty(self):
        self.assertEqual(compute_xor(b''), 0)

    def test_single_byte(self):
        self.assertEqual(compute_xor(bytes([0xAB])), 0xAB)

    def test_two_equal_bytes_cancel(self):
        self.assertEqual(compute_xor(bytes([0x55, 0x55])), 0x00)

    def test_known_values(self):
        # 0x06 ^ 0x02 ^ 0x00 ^ 0x64 ^ 0x00 ^ 0x00 ^ 0x00 ^ 0x00 = ?
        data = bytes([0x06, 0x02, 0x00, 0x64, 0x00, 0x00, 0x00, 0x00])
        expected = 0
        for b in data:
            expected ^= b
        self.assertEqual(compute_xor(data), expected)


class TestBuildMotionFrame(unittest.TestCase):
    """Tests for the UART frame builder."""

    def _parse_frame(self, frame: bytes):
        """Parse a frame and return its fields."""
        self.assertEqual(frame[0], FRAME_HEADER0)
        self.assertEqual(frame[1], FRAME_HEADER1)
        data_length = frame[2]
        cmd = frame[3]
        motion_data = frame[4:4 + data_length - 1]  # DataLength includes CMD byte
        xor_check = frame[-1]
        return data_length, cmd, motion_data, xor_check

    def test_frame_length(self):
        frame = build_motion_frame(100, 0, 0)
        # Header(2) + DataLength(1) + CMD(1) + 3×int16(6) + XOR(1) = 11 bytes
        self.assertEqual(len(frame), 11)

    def test_headers(self):
        frame = build_motion_frame(0, 0, 0)
        self.assertEqual(frame[0], 0x0A)
        self.assertEqual(frame[1], 0x0C)

    def test_cmd_byte(self):
        frame = build_motion_frame(0, 0, 0)
        self.assertEqual(frame[3], CMD_MOTION)  # 0x02

    def test_data_length_field(self):
        frame = build_motion_frame(0, 0, 0)
        self.assertEqual(frame[2], 0x06)

    def test_zero_velocity(self):
        frame = build_motion_frame(0, 0, 0)
        # Bytes 4..9 are three 16-bit signed integers, all zero
        vx, vy, w = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(vx, 0)
        self.assertEqual(vy, 0)
        self.assertEqual(w, 0)

    def test_positive_vx(self):
        frame = build_motion_frame(500, 0, 0)
        vx, vy, w = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(vx, 500)
        self.assertEqual(vy, 0)
        self.assertEqual(w, 0)

    def test_negative_values(self):
        frame = build_motion_frame(-300, -400, -1000)
        vx, vy, w = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(vx, -300)
        self.assertEqual(vy, -400)
        self.assertEqual(w, -1000)

    def test_xor_checksum_valid(self):
        frame = build_motion_frame(100, 200, 300)
        # XOR covers bytes from DataLength to last data byte (indices 2..9)
        checksum_payload = frame[2:10]
        expected_xor = compute_xor(checksum_payload)
        self.assertEqual(frame[10], expected_xor)

    def test_xor_checksum_zero_velocity(self):
        frame = build_motion_frame(0, 0, 0)
        checksum_payload = frame[2:10]
        self.assertEqual(frame[10], compute_xor(checksum_payload))

    def test_clamp_max_linear(self):
        frame = build_motion_frame(9999, 9999, 0)
        vx, vy, _ = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(vx, 2000)
        self.assertEqual(vy, 2000)

    def test_clamp_min_linear(self):
        frame = build_motion_frame(-9999, -9999, 0)
        vx, vy, _ = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(vx, -2000)
        self.assertEqual(vy, -2000)

    def test_clamp_max_angular(self):
        frame = build_motion_frame(0, 0, 99999)
        _, _, w = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(w, 6870)

    def test_clamp_min_angular(self):
        frame = build_motion_frame(0, 0, -99999)
        _, _, w = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(w, -6870)

    def test_boundary_max_values(self):
        frame = build_motion_frame(2000, 2000, 6870)
        vx, vy, w = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(vx, 2000)
        self.assertEqual(vy, 2000)
        self.assertEqual(w, 6870)

    def test_boundary_min_values(self):
        frame = build_motion_frame(-2000, -2000, -6870)
        vx, vy, w = struct.unpack('>hhh', frame[4:10])
        self.assertEqual(vx, -2000)
        self.assertEqual(vy, -2000)
        self.assertEqual(w, -6870)


class TestBuildCanMotionData(unittest.TestCase):
    """Tests for the CAN payload builder."""

    def test_payload_length(self):
        data = build_can_motion_data(0, 0, 0)
        self.assertEqual(len(data), 8)

    def test_cmd_byte(self):
        data = build_can_motion_data(0, 0, 0)
        self.assertEqual(data[0], 0x02)

    def test_zero_velocity(self):
        data = build_can_motion_data(0, 0, 0)
        vx, vy, w = struct.unpack('>hhh', data[1:7])
        self.assertEqual(vx, 0)
        self.assertEqual(vy, 0)
        self.assertEqual(w, 0)

    def test_positive_velocity(self):
        data = build_can_motion_data(1000, 500, 2000)
        vx, vy, w = struct.unpack('>hhh', data[1:7])
        self.assertEqual(vx, 1000)
        self.assertEqual(vy, 500)
        self.assertEqual(w, 2000)

    def test_negative_velocity(self):
        data = build_can_motion_data(-1000, -500, -2000)
        vx, vy, w = struct.unpack('>hhh', data[1:7])
        self.assertEqual(vx, -1000)
        self.assertEqual(vy, -500)
        self.assertEqual(w, -2000)

    def test_xor_checksum(self):
        data = build_can_motion_data(100, 200, 300)
        expected_xor = can_compute_xor(data[:7])
        self.assertEqual(data[7], expected_xor)

    def test_clamp_linear(self):
        data = build_can_motion_data(9999, -9999, 0)
        vx, vy, _ = struct.unpack('>hhh', data[1:7])
        self.assertEqual(vx, 2000)
        self.assertEqual(vy, -2000)

    def test_clamp_angular(self):
        data = build_can_motion_data(0, 0, 99999)
        _, _, w = struct.unpack('>hhh', data[1:7])
        self.assertEqual(w, 6870)


if __name__ == '__main__':
    unittest.main()
