#!/usr/bin/env python3
"""
Example: send motion commands directly to the LZ_OMNI chassis via UART.

This script demonstrates how to build and send protocol frames without ROS 2.
Run it with a connected chassis to verify serial communication.

Usage:
  python3 send_example.py
  python3 send_example.py --port /dev/ttyUSB1 --baudrate 115200
"""

import argparse
import time
import sys
import os

# Ensure the package is importable when running from source
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from lz_omni_chassis_driver.uart_driver import UartDriver, build_motion_frame
from lz_omni_chassis_driver.can_driver import CanDriver, build_can_motion_data

import logging
logging.basicConfig(level=logging.INFO, format='%(levelname)s: %(message)s')
log = logging.getLogger(__name__)


def demo_uart(port: str, baudrate: int) -> None:
    """Send a simple velocity sequence via UART."""
    driver = UartDriver(port=port, baudrate=baudrate)
    if not driver.open():
        log.error('Cannot open serial port – check connection and port name.')
        return

    log.info('=== UART send example ===')
    commands = [
        (500,  0,     0,    'Forward  0.5 m/s'),
        (-500, 0,     0,    'Backward 0.5 m/s'),
        (0,    500,   0,    'Strafe right 0.5 m/s'),
        (0,    -500,  0,    'Strafe left 0.5 m/s'),
        (0,    0,     1000, 'Rotate CCW 1 rad/s'),
        (0,    0,     0,    'Stop'),
    ]

    for vx, vy, w, label in commands:
        log.info(f'  {label}: vx={vx} mm/s, vy={vy} mm/s, w={w} × 0.001 rad/s')
        frame = build_motion_frame(vx, vy, w)
        log.info(f'  Frame: {frame.hex()}')
        driver.send_motion(vx, vy, w)
        time.sleep(1.0)

    driver.close()
    log.info('Done.')


def demo_can(channel: str) -> None:
    """Send a simple velocity sequence via CAN."""
    driver = CanDriver(channel=channel)
    if not driver.open():
        log.error(
            'Cannot open CAN interface – ensure SocketCAN is configured:\n'
            f'  sudo ip link set {channel} up type can bitrate 500000'
        )
        return

    log.info('=== CAN send example ===')
    commands = [
        (500,  0,    0,    'Forward  0.5 m/s'),
        (-500, 0,    0,    'Backward 0.5 m/s'),
        (0,    0,    1000, 'Rotate CCW 1 rad/s'),
        (0,    0,    0,    'Stop'),
    ]

    for vx, vy, w, label in commands:
        log.info(f'  {label}: vx={vx} mm/s, vy={vy} mm/s, w={w} × 0.001 rad/s')
        data = build_can_motion_data(vx, vy, w)
        log.info(f'  CAN data: {data.hex()}')
        driver.send_motion(vx, vy, w)
        time.sleep(1.0)

    driver.close()
    log.info('Done.')


def demo_frame_only() -> None:
    """Show protocol frames without a physical connection."""
    log.info('=== Protocol frame preview (no hardware required) ===')
    test_cases = [
        (500,   0,    0,    'Forward 0.5 m/s'),
        (0,     500,  0,    'Strafe 0.5 m/s'),
        (0,     0,    1000, 'Rotate 1 rad/s'),
        (2000,  2000, 6870, 'Maximum values'),
        (-2000, -2000, -6870, 'Minimum values'),
        (0,     0,    0,    'Zero / Stop'),
    ]
    for vx, vy, w, label in test_cases:
        uart_frame = build_motion_frame(vx, vy, w)
        can_data = build_can_motion_data(vx, vy, w)
        log.info(
            f'{label:30s} | UART: {uart_frame.hex()} | CAN: {can_data.hex()}'
        )


if __name__ == '__main__':
    parser = argparse.ArgumentParser(description='LZ_OMNI chassis send example')
    parser.add_argument(
        '--mode', choices=['uart', 'can', 'preview'], default='preview',
        help="'uart'=serial demo, 'can'=CAN demo, 'preview'=show frames without hardware"
    )
    parser.add_argument('--port', default='/dev/ttyUSB0', help='Serial port (UART mode)')
    parser.add_argument('--baudrate', type=int, default=115200, help='Baud rate (UART mode)')
    parser.add_argument('--can-channel', default='can0', help='CAN channel (CAN mode)')
    args = parser.parse_args()

    if args.mode == 'uart':
        demo_uart(args.port, args.baudrate)
    elif args.mode == 'can':
        demo_can(args.can_channel)
    else:
        demo_frame_only()
