#!/usr/bin/env python3
"""Preview or send documented custom CAN FD PDO and MIT commands."""

import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.can_driver import CanDriver
from juxiedrive.protocol import MultiAxisCommand, build_custom_command, build_custom_multi_command, build_mit_command

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
log = logging.getLogger(__name__)


def show_frame(name, frame):
    arbitration_id, data, is_fd = frame
    log.info('%s id=0x%03X is_fd=%s len=%d data=%s', name, arbitration_id, is_fd, len(data), data.hex())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='JuxieDrive custom PDO/MIT demo')
    parser.add_argument('--mode', choices=['preview', 'can'], default='preview')
    parser.add_argument('--can-channel', default='can0')
    args = parser.parse_args()

    single = build_custom_command(1, 'profile_position', target_1=0x09E1, target_2=2000, feedforward=10)
    multi = build_custom_multi_command([
        MultiAxisCommand(node_id=1, mode='profile_position', target_1=100, target_2=1000),
        MultiAxisCommand(node_id=2, mode='profile_velocity', target_1=500, target_2=2000),
        MultiAxisCommand(node_id=3, mode='current', target_1=250),
    ])
    mit = build_mit_command(1, position=10.0, velocity=120.0, kp=50.0, kd=1.5, torque=3.0)

    if args.mode == 'preview':
        show_frame('single_axis', single)
        show_frame('multi_axis', multi)
        show_frame('mit_mode', mit)
    else:
        driver = CanDriver(channel=args.can_channel)
        if not driver.open():
            raise SystemExit(1)
        try:
            for name, frame in [('single_axis', single), ('multi_axis', multi), ('mit_mode', mit)]:
                log.info('Sending %s', name)
                driver.send_frame(*frame)
        finally:
            driver.close()
