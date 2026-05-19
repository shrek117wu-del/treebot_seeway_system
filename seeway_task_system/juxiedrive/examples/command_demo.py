#!/usr/bin/env python3
"""Preview or send documented SDO/NMT commands for JuxieDrive."""

import argparse
import json
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.can_driver import CanDriver
from juxiedrive.protocol import (
    IDX_CANFD_DATA_BITRATE,
    IDX_DISABLE_WATCHDOG_LIMIT,
    IDX_HEARTBEAT_TIME,
    IDX_NODE_ID,
    IDX_SOFTWARE_LIMIT,
    IDX_ZERO_CALIBRATION,
    PI_INDEX_MAP,
    build_current_sequence,
    build_enable_sequence,
    build_profile_position_sequence,
    build_profile_velocity_sequence,
    build_query_state_sequence,
    build_query_version_sequence,
    build_sdo_write_request,
)

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
log = logging.getLogger(__name__)


def preview_frames(frames):
    for arbitration_id, data, is_fd in frames:
        log.info('id=0x%03X is_fd=%s data=%s', arbitration_id, is_fd, data.hex())


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='JuxieDrive documented SDO command demo')
    parser.add_argument('--mode', choices=['preview', 'can'], default='preview')
    parser.add_argument('--can-channel', default='can0')
    parser.add_argument('--node-id', type=int, default=1)
    args = parser.parse_args()

    node_id = args.node_id
    sequences = {
        'query_version': build_query_version_sequence(node_id),
        'enable': build_enable_sequence(node_id),
        'profile_position': build_profile_position_sequence(node_id, 0x4000, 10, 2000, 2000),
        'profile_velocity': build_profile_velocity_sequence(node_id, 500, 1000, 1000),
        'current': build_current_sequence(node_id, 500),
        'query_state': build_query_state_sequence(node_id),
        'set_node_id': [build_sdo_write_request(node_id, IDX_NODE_ID, 0, 5, 4)],
        'set_zero_position': [build_sdo_write_request(node_id, IDX_ZERO_CALIBRATION, 0, 1, 4)],
        'disable_watchdog_and_limits': [build_sdo_write_request(node_id, IDX_DISABLE_WATCHDOG_LIMIT, 0, 1, 4)],
        'set_heartbeat': [build_sdo_write_request(node_id, IDX_HEARTBEAT_TIME, 0, 2000, 2)],
        'set_positive_limit': [build_sdo_write_request(node_id, IDX_SOFTWARE_LIMIT, 2, 0x7F30, 4)],
        'set_negative_limit': [build_sdo_write_request(node_id, IDX_SOFTWARE_LIMIT, 1, -0x7F38, 4, signed=True)],
        'set_canfd_bitrate': [build_sdo_write_request(node_id, IDX_CANFD_DATA_BITRATE, 0, 1, 4)],
        'set_pi': [build_sdo_write_request(node_id, PI_INDEX_MAP['speed_kp'], 0, 123, 4)],
    }

    if args.mode == 'preview':
        for name, frames in sequences.items():
            log.info('=== %s ===', name)
            preview_frames(frames)
        log.info('JSON topic example: %s', json.dumps({'type': 'profile_velocity', 'node_id': node_id, 'target_velocity_rpm': 500, 'acceleration': 1000, 'deceleration': 1000}, ensure_ascii=False))
    else:
        driver = CanDriver(channel=args.can_channel)
        if not driver.open():
            raise SystemExit(1)
        try:
            for name, frames in sequences.items():
                log.info('Sending %s', name)
                driver.send_sequence(frames)
        finally:
            driver.close()
