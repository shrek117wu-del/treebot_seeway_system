#!/usr/bin/env python3
"""Preview or send the documented CANopen PDO mapping sequence."""

import argparse
import logging
import os
import sys

sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from juxiedrive.can_driver import CanDriver
from juxiedrive.protocol import build_canopen_pdo_mapping_sequence, build_nmt_start_frame, build_rpdo1_frame, build_sync_frame

logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
log = logging.getLogger(__name__)


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='JuxieDrive PDO mapping demo')
    parser.add_argument('--mode', choices=['preview', 'can'], default='preview')
    parser.add_argument('--can-channel', default='can0')
    parser.add_argument('--node-id', type=int, default=1)
    args = parser.parse_args()

    frames = [build_nmt_start_frame(args.node_id)]
    frames.extend(build_canopen_pdo_mapping_sequence(args.node_id))
    frames.append(build_sync_frame())
    frames.append(build_rpdo1_frame(args.node_id, 0x000F, 100, 3641))

    if args.mode == 'preview':
        for arbitration_id, data, is_fd in frames:
            log.info('id=0x%03X is_fd=%s data=%s', arbitration_id, is_fd, data.hex())
    else:
        driver = CanDriver(channel=args.can_channel)
        if not driver.open():
            raise SystemExit(1)
        try:
            driver.send_sequence(frames)
        finally:
            driver.close()
