#!/usr/bin/env python3
"""CAN driver for the LZ_OMNI omnidirectional chassis."""

import logging
import struct
import threading
import time
from typing import Callable, Dict, Optional

from .protocol import (
    AuxInfo,
    BatteryInfo,
    ChassisStatus,
    VersionInfo,
    CMD_MOTOR_CONTROL,
    CMD_MOTION_OMNI,
    CMD_VERSION_QUERY,
    compute_xor,
)

DEFAULT_CAN_ID = 0x001
CMD_MOTION = 0x02
DEFAULT_RX_CAN_IDS: Dict[int, str] = {
    0x69: 'battery',
    0x6B: 'chassis',
    0x6F: 'aux',
    0x11: 'version',
}


def build_can_motion_data(linear_x_mms: int, linear_y_mms: int, angular_mrad_s: int) -> bytes:
    """Build legacy CMD 0x02 CAN motion payload for backward compatibility tests."""
    vx = max(-2000, min(2000, linear_x_mms))
    vy = max(-2000, min(2000, linear_y_mms))
    vw = max(-6870, min(6870, angular_mrad_s))
    motion_bytes = struct.pack('>hhh', vx, vy, vw)
    payload_no_check = bytes([CMD_MOTION]) + motion_bytes
    return payload_no_check + bytes([compute_xor(payload_no_check)])


class CanDriver:
    """SocketCAN driver implementing TX/RX framing for the LZ_OMNI protocol."""

    def __init__(
        self,
        channel: str = 'can0',
        bitrate: int = 500000,
        can_id: int = DEFAULT_CAN_ID,
        rx_can_ids: Optional[Dict[int, str]] = None,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """Initialise CAN bus parameters and parser callbacks."""
        self._channel = channel
        self._bitrate = bitrate
        self._can_id = can_id
        self._rx_can_ids = dict(rx_can_ids or DEFAULT_RX_CAN_IDS)
        self._logger = logger or logging.getLogger(__name__)

        self._bus = None
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None

        self._battery_info_callback: Optional[Callable[[BatteryInfo], None]] = None
        self._chassis_status_callback: Optional[Callable[[ChassisStatus], None]] = None
        self._aux_info_callback: Optional[Callable[[AuxInfo], None]] = None
        self._version_info_callback: Optional[Callable[[VersionInfo], None]] = None

    def open(self) -> bool:
        """Open the SocketCAN interface and start receiving thread."""
        try:
            import can

            self._bus = can.interface.Bus(
                channel=self._channel,
                bustype='socketcan',
                bitrate=self._bitrate,
            )
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._logger.info(
                f'CAN bus opened: {self._channel} @ {self._bitrate} bps, tx_id=0x{self._can_id:03X}'
            )
            return True
        except Exception as exc:
            self._logger.error(f'Failed to open CAN {self._channel}: {exc}')
            return False

    def close(self) -> None:
        """Shut down CAN receiver thread and close interface."""
        self._running = False
        if self._read_thread is not None:
            self._read_thread.join(timeout=2.0)
        with self._lock:
            if self._bus is not None:
                try:
                    self._bus.shutdown()
                except Exception:
                    pass
                self._bus = None
                self._logger.info('CAN bus closed')

    def is_open(self) -> bool:
        """Return ``True`` when CAN bus object is available."""
        return self._bus is not None

    def send_motion(self, linear_x_mms: int, linear_y_mms: int, angular_mrad_s: int) -> bool:
        """Send CMD 0x40 omnidirectional velocity command on CAN."""
        vx = max(-2000, min(2000, linear_x_mms))
        vy = max(-2000, min(2000, linear_y_mms))
        vw = max(-6870, min(6870, angular_mrad_s))
        payload = struct.pack('>hhh', vx, vy, vw)
        return self._send_can_command(
            CMD_MOTION_OMNI,
            payload,
            f'CAN TX CMD=0x40 vx={linear_x_mms} vy={linear_y_mms} vw={angular_mrad_s}',
        )

    def send_motor_control(self, m1: int, m2: int, m3: int, m4: int) -> bool:
        """Send CMD 0x01 motor control command.

        Note: CAN2.0 payload is limited to 6 bytes after CMD; this command needs 8 bytes.
        The method logs an error and returns False when frame size exceeds CAN2.0 limit.
        """
        payload = struct.pack(
            '>hhhh',
            max(-3000, min(3000, m1)),
            max(-3000, min(3000, m2)),
            max(-3000, min(3000, m3)),
            max(-3000, min(3000, m4)),
        )
        return self._send_can_command(CMD_MOTOR_CONTROL, payload, f'CAN TX CMD=0x01 m=[{m1},{m2},{m3},{m4}]')

    def send_version_query(self) -> bool:
        """Send CMD 0x11 firmware version query on configured TX CAN ID."""
        return self._send_can_command(CMD_VERSION_QUERY, b'', 'CAN TX CMD=0x11 version query')

    def set_battery_info_callback(self, callback: Optional[Callable[[BatteryInfo], None]]) -> None:
        """Register callback for parsed battery telemetry."""
        self._battery_info_callback = callback

    def set_chassis_status_callback(self, callback: Optional[Callable[[ChassisStatus], None]]) -> None:
        """Register callback for parsed chassis status telemetry."""
        self._chassis_status_callback = callback

    def set_aux_info_callback(self, callback: Optional[Callable[[AuxInfo], None]]) -> None:
        """Register callback for parsed auxiliary telemetry."""
        self._aux_info_callback = callback

    def set_version_info_callback(self, callback: Optional[Callable[[VersionInfo], None]]) -> None:
        """Register callback for parsed firmware version responses."""
        self._version_info_callback = callback

    def _send_can_command(self, cmd: int, payload: bytes, debug_message: str) -> bool:
        """Send a CAN command using ``[CMD][D0..D5][XOR]`` 8-byte frame format."""
        if not self.is_open():
            self._logger.warning('CAN not open – cannot send command')
            return False
        if len(payload) > 6:
            self._logger.error(
                f'CAN payload too long for CMD=0x{cmd:02X}: {len(payload)} bytes (max 6 for CAN2.0 frame)'
            )
            return False

        data_0_6 = bytes([cmd]) + payload + bytes(6 - len(payload))
        checksum = compute_xor(data_0_6)
        frame_data = data_0_6 + bytes([checksum])

        try:
            import can

            msg = can.Message(
                arbitration_id=self._can_id,
                data=frame_data,
                is_extended_id=False,
            )
            with self._lock:
                self._bus.send(msg)
            self._logger.debug(f'{debug_message} frame={frame_data.hex()}')
            return True
        except Exception as exc:
            self._logger.error(f'CAN send error: {exc}')
            return False

    def _read_loop(self) -> None:
        """Receive CAN messages and dispatch parsed payloads using RX CAN IDs."""
        while self._running:
            try:
                if self._bus is None:
                    time.sleep(0.1)
                    continue
                msg = self._bus.recv(timeout=0.1)
                if msg is None:
                    continue

                self._logger.debug(f'CAN RX id=0x{msg.arbitration_id:03X} data={bytes(msg.data).hex()}')
                rx_type = self._rx_can_ids.get(msg.arbitration_id)
                if rx_type is None:
                    continue

                if len(msg.data) < 2:
                    self._logger.error(f'CAN RX frame too short id=0x{msg.arbitration_id:03X}')
                    continue

                data_bytes = bytes(msg.data)
                if len(data_bytes) >= 8:
                    expected_xor = compute_xor(data_bytes[:7])
                    if data_bytes[7] != expected_xor:
                        self._logger.error(
                            f'CAN RX XOR mismatch id=0x{msg.arbitration_id:03X} '
                            f'got=0x{data_bytes[7]:02X} exp=0x{expected_xor:02X}'
                        )
                        continue

                expected_lengths = {
                    'battery': 6,
                    'chassis': 14,
                    'aux': 8,
                    'version': 4,
                }
                data_len = expected_lengths.get(rx_type)
                if data_len is None:
                    self._logger.error(f'CAN RX unsupported type={rx_type}')
                    continue
                payload = data_bytes[1:1 + data_len]
                self._dispatch(rx_type, payload)
            except Exception as exc:
                if self._running:
                    self._logger.error(f'CAN read error: {exc}')
                time.sleep(0.1)

    def _dispatch(self, rx_type: str, payload: bytes) -> None:
        """Parse payload by RX frame type and invoke matching callback."""
        try:
            if rx_type == 'battery':
                info = BatteryInfo.from_bytes(payload)
                if self._battery_info_callback:
                    self._battery_info_callback(info)
            elif rx_type == 'chassis':
                info = ChassisStatus.from_bytes(payload)
                if self._chassis_status_callback:
                    self._chassis_status_callback(info)
            elif rx_type == 'aux':
                info = AuxInfo.from_bytes(payload)
                if self._aux_info_callback:
                    self._aux_info_callback(info)
            elif rx_type == 'version':
                info = VersionInfo.from_bytes(payload)
                if self._version_info_callback:
                    self._version_info_callback(info)
        except Exception as exc:
            self._logger.error(f'CAN RX parse/dispatch error type={rx_type}: {exc}')
