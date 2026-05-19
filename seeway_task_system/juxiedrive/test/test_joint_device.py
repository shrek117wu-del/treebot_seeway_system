#!/usr/bin/env python3
"""Unit tests for the JointDevice abstraction."""

import os
import sys
import unittest

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.can_driver import CanFrame
from juxiedrive.joint_device import JointDevice
from juxiedrive.protocol import SDO_RX_BASE
from juxiedrive.state_types import JointConfig


class FakeTransport:
    def __init__(self):
        self.sent = []

    def send(self, arbitration_id, data, is_fd=False):
        self.sent.append((arbitration_id, bytes(data), is_fd))
        return True


class TestJointDevice(unittest.TestCase):
    def setUp(self):
        self.transport = FakeTransport()
        self.device = JointDevice(JointConfig(name='joint1', node_id=1), self.transport)

    def test_feedback_frame_updates_runtime_state(self):
        handled = self.device.handle_frame(CanFrame(0x301, bytes.fromhex('09e1fde5ff51000000f001c0'), is_fd=True))
        self.assertTrue(handled)
        self.assertAlmostEqual(self.device.state.position_deg, 13.8922, delta=0.01)
        self.assertEqual(self.device.state.velocity_rpm, -539.0)
        self.assertTrue(self.device.state.enabled)
        self.assertTrue(self.device.state.brake_released)

    def test_sdo_response_updates_position_state(self):
        handled = self.device.handle_frame(CanFrame(SDO_RX_BASE + 1, bytes.fromhex('43646000fe3f0000')))
        self.assertTrue(handled)
        self.assertAlmostEqual(self.device.state.position_deg, 89.99, delta=0.5)

    def test_enable_sequence_sends_three_frames(self):
        success = self.device.enable()
        self.assertTrue(success)
        self.assertEqual(len(self.transport.sent), 3)
        self.assertEqual(self.transport.sent[0][1], bytes.fromhex('2b40600006000000'))

    def test_single_axis_command_sends_canfd_frame(self):
        result = self.device.execute_named_command('single_pdo', {
            'enable': True,
            'release_brake': True,
            'clear_error': False,
            'control_mode': 1,
            'target_param_1': 0x16E5,
        })
        self.assertTrue(result['success'])
        self.assertEqual(self.transport.sent[-1][0], 0x101)
        self.assertTrue(self.transport.sent[-1][2])


if __name__ == '__main__':
    unittest.main()
