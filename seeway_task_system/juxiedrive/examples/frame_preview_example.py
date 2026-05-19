#!/usr/bin/env python3
"""Preview the documented JuxieDrive frames without requiring hardware."""

import json
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.protocol import (
    ControlMode,
    MitCommand,
    SingleAxisCommand,
    build_current_mode_sequence,
    build_mit_fd_command,
    build_multi_axis_fd_command,
    build_profile_position_sequence,
    build_profile_velocity_sequence,
    build_set_heartbeat_command,
    build_single_axis_fd_command,
    build_status_read_sequence,
    build_version_read_sequence,
)


def show(label, frames):
    print(f'\n[{label}]')
    for arbitration_id, data, is_fd in frames:
        print(f'  id=0x{arbitration_id:03X} fd={is_fd} data={data.hex()}')


def main() -> None:
    show('version reads', build_version_read_sequence(1))
    show('status reads', build_status_read_sequence(1))
    show('profile position', build_profile_position_sequence(1, 90.0, 10, 2000, 2000))
    show('profile velocity', build_profile_velocity_sequence(1, 500, 1000, 1000))
    show('current mode', build_current_mode_sequence(1, 500))
    show('heartbeat', [build_set_heartbeat_command(1, 2000)])
    show(
        'single-axis CAN-FD',
        [build_single_axis_fd_command(1, SingleAxisCommand(True, True, False, ControlMode.PROFILE_POSITION, 0x16E5, 0, 0))],
    )
    show(
        'MIT CAN-FD',
        [build_mit_fd_command(1, MitCommand(True, True, False, 10.0, -500.0, 100.0, 1.5, 5.0, 180.0, 3030.0, 45.0))],
    )
    show(
        'broadcast CAN-FD',
        [
            build_multi_axis_fd_command([
                (1, SingleAxisCommand(True, True, False, ControlMode.PROFILE_POSITION, 0x16E5)),
                (2, SingleAxisCommand(True, True, False, ControlMode.PROFILE_POSITION, 0x2D15)),
            ])
        ],
    )


if __name__ == '__main__':
    main()
