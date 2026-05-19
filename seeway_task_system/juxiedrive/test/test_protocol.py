import math
import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.protocol import (
    CANOPEN_MODE_MIT,
    CANOPEN_MODE_PP,
    CUSTOM_FEEDBACK_BASE,
    HEARTBEAT_BASE,
    SDO_RX_BASE,
    TPDO1_BASE,
    MultiAxisCommand,
    build_canopen_pdo_mapping_sequence,
    build_custom_command,
    build_custom_multi_command,
    build_mit_command,
    build_profile_position_sequence,
    build_query_version_sequence,
    build_rpdo1_frame,
    build_sdo_read_request,
    build_sdo_write_request,
    parse_custom_feedback,
    parse_heartbeat,
    parse_sdo_response,
    parse_tpdo1_feedback,
)


class ProtocolTests(unittest.TestCase):
    def test_sdo_read_request(self):
        arbitration_id, data, is_fd = build_sdo_read_request(1, 0x6064, 0)
        self.assertEqual(arbitration_id, 0x601)
        self.assertFalse(is_fd)
        self.assertEqual(data, bytes.fromhex('4064600000000000'))

    def test_sdo_write_request(self):
        arbitration_id, data, is_fd = build_sdo_write_request(1, 0x6040, 0, 0x000F, 2)
        self.assertEqual(arbitration_id, 0x601)
        self.assertFalse(is_fd)
        self.assertEqual(data, bytes.fromhex('2b4060000f000000'))

    def test_query_version_sequence(self):
        frames = build_query_version_sequence(1)
        self.assertEqual(len(frames), 4)
        self.assertEqual(frames[0][1], bytes.fromhex('4008100000000000'))
        self.assertEqual(frames[3][1], bytes.fromhex('4009100000000000'))

    def test_profile_position_sequence(self):
        frames = build_profile_position_sequence(1, 0x4000, 10, 2000, 2000)
        self.assertEqual(frames[0][1], bytes.fromhex('2b40600006000000'))
        self.assertEqual(frames[1][1], bytes.fromhex('2b40600007000000'))
        self.assertEqual(frames[2][1], bytes.fromhex('2b4060000f000000'))
        self.assertEqual(frames[3][1], bytes.fromhex('2f60600001000000'))
        self.assertEqual(frames[-1][1], bytes.fromhex('2b4060004f000000'))

    def test_custom_single_axis_command(self):
        arbitration_id, data, is_fd = build_custom_command(1, 'profile_position', 100, 2000, 10)
        self.assertEqual(arbitration_id, 0x101)
        self.assertTrue(is_fd)
        self.assertEqual(len(data), 7)
        self.assertEqual(data[0], 0xC2)
        self.assertEqual(data[1:], bytes.fromhex('006407d0000a'))

    def test_custom_multi_axis_command(self):
        arbitration_id, data, is_fd = build_custom_multi_command([
            MultiAxisCommand(node_id=1, mode='profile_position', target_1=100),
            MultiAxisCommand(node_id=2, mode='profile_velocity', target_1=200),
        ])
        self.assertEqual(arbitration_id, 0x200)
        self.assertTrue(is_fd)
        self.assertEqual(len(data), 64)
        self.assertEqual(data[56], 1)
        self.assertEqual(data[57], 2)

    def test_mit_command(self):
        arbitration_id, data, is_fd = build_mit_command(1, 0.0, 0.0, 0.0, 0.0, 0.0)
        self.assertEqual(arbitration_id, 0x111)
        self.assertTrue(is_fd)
        self.assertEqual(len(data), 9)
        self.assertEqual(data[0] >> 1 & 0x0F, CANOPEN_MODE_MIT)

    def test_parse_sdo_response(self):
        response = parse_sdo_response(SDO_RX_BASE + 1, bytes.fromhex('43646000fe3f0000'))
        self.assertEqual(response.node_id, 1)
        self.assertEqual(response.index, 0x6064)
        self.assertEqual(response.value_unsigned, 0x3FFE)

    def test_parse_heartbeat(self):
        info = parse_heartbeat(HEARTBEAT_BASE + 1, b'\x00')
        self.assertTrue(info.is_bootup)

    def test_parse_custom_feedback(self):
        # Example from the protocol manual: position=0x09E1, velocity=-539 RPM,
        # current=-175 mA, no fault, temperature=24.0 C, PP mode, enabled.
        feedback = parse_custom_feedback(CUSTOM_FEEDBACK_BASE + 1, bytes.fromhex('09e1fde5ff51000000f001c0'))
        self.assertAlmostEqual(feedback.position_deg, 13.892, delta=0.01)
        self.assertEqual(feedback.velocity_rpm, -539)
        self.assertEqual(feedback.current_ma, -175)
        self.assertAlmostEqual(feedback.coil_temperature_c, 24.0, delta=0.01)
        self.assertEqual(feedback.mode, CANOPEN_MODE_PP)
        self.assertTrue(feedback.enabled)
        self.assertTrue(feedback.brake_released)
        self.assertFalse(feedback.fault)

    def test_parse_tpdo1_feedback(self):
        feedback = parse_tpdo1_feedback(TPDO1_BASE + 1, bytes.fromhex('27004501fe3f0000'))
        self.assertEqual(feedback.status_word, 0x0027)
        self.assertEqual(feedback.actual_current_ma, 325)
        self.assertEqual(feedback.actual_position_counts, 0x3FFE)
        self.assertAlmostEqual(feedback.actual_position_deg, 89.99, delta=0.1)

    def test_rpdo1_frame(self):
        arbitration_id, data, is_fd = build_rpdo1_frame(1, 0x000F, 100, 3641)
        self.assertEqual(arbitration_id, 0x201)
        self.assertFalse(is_fd)
        self.assertEqual(data, bytes.fromhex('0f006400390e0000'))

    def test_pdo_mapping_sequence_contains_mode_switch(self):
        frames = build_canopen_pdo_mapping_sequence(1)
        self.assertGreater(len(frames), 10)
        self.assertEqual(frames[-1][1], bytes.fromhex('2f60600008000000'))


if __name__ == '__main__':
    unittest.main()
