#!/usr/bin/env python3
"""Dry-run example showing the documented high-level command sequences."""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.protocol import (
    build_disable_watchdog_and_limit_command,
    build_profile_position_sequence,
    build_profile_velocity_sequence,
    build_current_mode_sequence,
    build_set_canfd_bitrate_command,
    build_set_heartbeat_command,
    build_set_id_command,
    build_set_limit_command,
    build_set_pi_command,
    build_zero_calibration_command,
)


def print_sequence(title, frames):
    print(f'\n{title}')
    for arbitration_id, data, is_fd in frames:
        print(f'  id=0x{arbitration_id:03X} fd={is_fd} data={data.hex()}')


def main() -> None:
    print_sequence('Set joint ID to 5', [build_set_id_command(1, 5)])
    print_sequence('Zero calibration', [build_zero_calibration_command(1)])
    print_sequence('Disable watchdog and limits', [build_disable_watchdog_and_limit_command(1)])
    print_sequence('Set heartbeat 2000ms', [build_set_heartbeat_command(1, 2000)])
    print_sequence('Set positive limit to +179deg', [build_set_limit_command(1, 179.0, positive=True)])
    print_sequence('Set negative limit to -179deg', [build_set_limit_command(1, -179.0, positive=False)])
    print_sequence('Set current loop P', [build_set_pi_command(1, 0x2532, 1234)])
    print_sequence('Profile position 90deg', build_profile_position_sequence(1, 90.0, 10, 2000, 2000))
    print_sequence('Profile velocity 500rpm', build_profile_velocity_sequence(1, 500, 1000, 1000))
    print_sequence('Current mode 500mA', build_current_mode_sequence(1, 500))
    print_sequence('Set CAN-FD data bitrate code=1', [build_set_canfd_bitrate_command(1, 1)])


if __name__ == '__main__':
    main()
