#!/usr/bin/env python3
"""Unit tests for the JuxieDrive protocol helpers."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.protocol import (
    ControlMode,
    JointFeedback,
    MitCommand,
    SingleAxisCommand,
    build_mit_fd_command,
    build_multi_axis_fd_command,
    build_profile_position_sequence,
    build_sdo_read,
    build_sdo_write_u32,
    build_set_heartbeat_command,
    build_single_axis_fd_command,
    build_sync_frame,
    build_version_read_sequence,
    degrees_to_position_counts,
    parse_joint_feedback,
    parse_sdo_response,
    position_counts_to_degrees,
)


class TestSdoBuilders(unittest.TestCase):
    def test_build_sdo_read(self):
        arbitration_id, data, is_fd = build_sdo_read(1, 0x6064, 0x00)
        self.assertEqual(arbitration_id, 0x601)
        self.assertFalse(is_fd)
        self.assertEqual(data, bytes.fromhex('4064600000000000'))

    def test_build_sdo_write_u32(self):
        arbitration_id, data, is_fd = build_sdo_write_u32(5, 0x2530, 0x00, 5)
        self.assertEqual(arbitration_id, 0x605)
        self.assertFalse(is_fd)
        self.assertEqual(data, bytes.fromhex('2330250005000000'))

    def test_version_read_sequence_has_all_queries(self):
        frames = build_version_read_sequence(1)
        self.assertEqual(len(frames), 4)
        self.assertEqual(frames[0][1], bytes.fromhex('4008100000000000'))
        self.assertEqual(frames[1][1], bytes.fromhex('4000100000000000'))
        self.assertEqual(frames[2][1], bytes.fromhex('400a100000000000'))
        self.assertEqual(frames[3][1], bytes.fromhex('4009100000000000'))

    def test_profile_position_sequence_matches_documented_enable_prefix(self):
        frames = build_profile_position_sequence(1, 90.0, 10, 2000, 2000)
        self.assertEqual(frames[0][1], bytes.fromhex('2b40600006000000'))
        self.assertEqual(frames[1][1], bytes.fromhex('2b40600007000000'))
        self.assertEqual(frames[2][1], bytes.fromhex('2b4060000f000000'))


class TestCanFdBuilders(unittest.TestCase):
    def test_single_axis_fd_builder(self):
        arbitration_id, data, is_fd = build_single_axis_fd_command(
            1,
            SingleAxisCommand(
                enable=True,
                release_brake=True,
                clear_error=False,
                control_mode=ControlMode.PROFILE_POSITION,
                target_param_1=0x16E5,
                target_param_2=0,
                feedforward=0,
            ),
        )
        self.assertEqual(arbitration_id, 0x101)
        self.assertTrue(is_fd)
        self.assertEqual(len(data), 7)
        self.assertEqual(data[0], 0xC2)

    def test_multi_axis_fd_builder(self):
        arbitration_id, data, is_fd = build_multi_axis_fd_command([
            (1, SingleAxisCommand(True, True, False, ControlMode.PROFILE_POSITION, 0x16E5)),
            (2, SingleAxisCommand(True, True, False, ControlMode.PROFILE_POSITION, 0x2D15)),
        ])
        self.assertEqual(arbitration_id, 0x200)
        self.assertTrue(is_fd)
        self.assertEqual(len(data), 64)
        self.assertEqual(data[56:58], bytes([1, 2]))

    def test_mit_fd_builder(self):
        arbitration_id, data, is_fd = build_mit_fd_command(
            1,
            MitCommand(True, True, False, 10.0, -500.0, 100.0, 1.5, 5.0, 180.0, 3030.0, 45.0),
        )
        self.assertEqual(arbitration_id, 0x111)
        self.assertTrue(is_fd)
        self.assertEqual(len(data), 9)
        self.assertEqual(data[0] & 0xFE, 0xCC)

    def test_sync_frame(self):
        arbitration_id, data, is_fd = build_sync_frame()
        self.assertEqual(arbitration_id, 0x80)
        self.assertEqual(data, b'')
        self.assertFalse(is_fd)

    def test_set_heartbeat_command(self):
        arbitration_id, data, is_fd = build_set_heartbeat_command(1, 2000)
        self.assertEqual(arbitration_id, 0x601)
        self.assertEqual(data, bytes.fromhex('2b171000d0070000'))
        self.assertFalse(is_fd)


class TestParsers(unittest.TestCase):
    def test_parse_joint_feedback(self):
        feedback = parse_joint_feedback(0x301, bytes.fromhex('09e1fde5ff51000000f001c0'))
        self.assertIsInstance(feedback, JointFeedback)
        self.assertAlmostEqual(feedback.position_deg, 13.8922, delta=0.01)
        self.assertEqual(feedback.velocity_rpm, -539.0)
        self.assertEqual(feedback.current_ma, -175.0)
        self.assertEqual(feedback.error_code, 0)
        self.assertEqual(feedback.coil_temperature_c, 24.0)
        self.assertEqual(feedback.control_mode, 1)
        self.assertTrue(feedback.enabled)
        self.assertTrue(feedback.brake_released)

    def test_parse_sdo_response(self):
        response = parse_sdo_response(0x581, bytes.fromhex('43646000fe3f0000'))
        self.assertEqual(response.node_id, 1)
        self.assertEqual(response.index, 0x6064)
        self.assertEqual(response.as_unsigned(), 16382)

    def test_position_conversion_roundtrip(self):
        counts = degrees_to_position_counts(90.0)
        self.assertEqual(counts, 16384)
        self.assertAlmostEqual(position_counts_to_degrees(counts), 90.0, delta=0.1)


if __name__ == '__main__':
    unittest.main()
