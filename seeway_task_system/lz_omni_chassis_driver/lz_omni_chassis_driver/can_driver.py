#!/usr/bin/env python3
"""
CAN Driver for LZ_OMNI Omnidirectional Chassis
Handles CAN 2.0B communication using SocketCAN (Linux).

CAN frame specification:
  Standard: CAN 2.0B, 500 kbps
  DLC: 8 bytes
  Data[0]: cmd = 0x02 (motion control)
  Data[1..2]: Linear-X (signed 16-bit big-endian, mm/s)
  Data[3..4]: Linear-Y (signed 16-bit big-endian, mm/s)
  Data[5..6]: Angular-Speed (signed 16-bit big-endian, 0.001 rad/s)
  Data[7]: XOR checksum of Data[0..6]
"""

import struct
import threading
import time
import logging
from typing import Optional


CMD_MOTION = 0x02
DEFAULT_CAN_ID = 0x001  # CAN arbitration ID for motion commands


def compute_xor(data: bytes) -> int:
    """Compute XOR checksum over the given bytes."""
    result = 0
    for b in data:
        result ^= b
    return result


def build_can_motion_data(
    linear_x_mms: int,
    linear_y_mms: int,
    angular_mrad_s: int,
) -> bytes:
    """
    Build 8-byte CAN data payload for a motion command.

    Args:
        linear_x_mms: X-axis speed in mm/s, range -2000..2000
        linear_y_mms: Y-axis speed in mm/s, range -2000..2000
        angular_mrad_s: Angular speed in 0.001 rad/s, range -6870..6870

    Returns:
        8-byte payload (cmd + motion data + XOR checksum).
    """
    # Clamp values to protocol limits
    linear_x_mms = max(-2000, min(2000, linear_x_mms))
    linear_y_mms = max(-2000, min(2000, linear_y_mms))
    angular_mrad_s = max(-6870, min(6870, angular_mrad_s))

    cmd = CMD_MOTION
    motion_bytes = struct.pack('>hhh', linear_x_mms, linear_y_mms, angular_mrad_s)
    payload_no_check = bytes([cmd]) + motion_bytes  # 7 bytes
    xor_check = compute_xor(payload_no_check)
    return payload_no_check + bytes([xor_check])  # 8 bytes total


class CanDriver:
    """
    CAN bus driver for the LZ_OMNI chassis using the Linux SocketCAN interface.

    Requires the `python-can` library and a configured SocketCAN interface
    (e.g. `sudo ip link set can0 up type can bitrate 500000`).
    """

    def __init__(
        self,
        channel: str = 'can0',
        bitrate: int = 500000,
        can_id: int = DEFAULT_CAN_ID,
        logger: Optional[logging.Logger] = None,
    ):
        self._channel = channel
        self._bitrate = bitrate
        self._can_id = can_id
        self._logger = logger or logging.getLogger(__name__)

        self._bus = None
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._rx_callback = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def open(self) -> bool:
        """Open the CAN interface.  Returns True on success."""
        try:
            import can  # lazy import so the module is loadable without python-can
            self._bus = can.interface.Bus(
                channel=self._channel,
                bustype='socketcan',
                bitrate=self._bitrate,
            )
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._logger.info(
                f'CAN bus opened: {self._channel} @ {self._bitrate} bps, '
                f'arbitration_id=0x{self._can_id:03X}'
            )
            return True
        except Exception as exc:
            self._logger.error(f'Failed to open CAN {self._channel}: {exc}')
            return False

    def close(self) -> None:
        """Shut down the CAN interface and stop the read thread."""
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
        """Return True if the CAN bus is currently open."""
        return self._bus is not None

    def set_rx_callback(self, callback) -> None:
        """Register a callable(msg) invoked for each received CAN message."""
        self._rx_callback = callback

    def send_motion(
        self,
        linear_x_mms: int,
        linear_y_mms: int,
        angular_mrad_s: int,
    ) -> bool:
        """
        Send a motion command via CAN.

        Args:
            linear_x_mms: X speed in mm/s
            linear_y_mms: Y speed in mm/s
            angular_mrad_s: Angular speed in 0.001 rad/s

        Returns:
            True if the CAN frame was sent successfully.
        """
        if not self.is_open():
            self._logger.warning('CAN not open – cannot send motion command')
            return False

        data = build_can_motion_data(linear_x_mms, linear_y_mms, angular_mrad_s)
        try:
            import can
            msg = can.Message(
                arbitration_id=self._can_id,
                data=data,
                is_extended_id=False,
            )
            with self._lock:
                self._bus.send(msg)
            self._logger.debug(
                f'CAN TX: vx={linear_x_mms} vy={linear_y_mms} w={angular_mrad_s} '
                f'data={data.hex()}'
            )
            return True
        except Exception as exc:
            self._logger.error(f'CAN send error: {exc}')
            return False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        """Background thread that receives CAN messages from the chassis."""
        while self._running:
            try:
                if self._bus is None:
                    time.sleep(0.1)
                    continue
                msg = self._bus.recv(timeout=0.1)
                if msg is not None and self._rx_callback:
                    self._rx_callback(msg)
            except Exception as exc:
                if self._running:
                    self._logger.error(f'CAN read error: {exc}')
                time.sleep(0.1)
