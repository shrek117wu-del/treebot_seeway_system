#!/usr/bin/env python3
"""
UART Driver for LZ_OMNI Omnidirectional Chassis
Handles serial communication using the chassis protocol.

Protocol frame format:
  [Header0][Header1][DataLength][CMD][Data...][XOR_Check]
  Header0: 0x0A
  Header1: 0x0C
  XOR_Check: XOR of all bytes from DataLength onward (excluding Header0/Header1 and XOR itself)

Motion control command (CMD=0x02):
  DataLength: 0x06
  Data: Linear-X (2B, mm/s), Linear-Y (2B, mm/s), Angular-Speed (2B, 0.001 rad/s)
"""

import struct
import threading
import time
import logging
from typing import Optional


FRAME_HEADER0 = 0x0A
FRAME_HEADER1 = 0x0C
CMD_MOTION = 0x02


def compute_xor(data: bytes) -> int:
    """Compute XOR checksum over the given bytes."""
    result = 0
    for b in data:
        result ^= b
    return result


def build_motion_frame(linear_x_mms: int, linear_y_mms: int, angular_mrad_s: int) -> bytes:
    """
    Build a motion control frame for the chassis.

    Args:
        linear_x_mms: X-axis speed in mm/s, range -2000..2000
        linear_y_mms: Y-axis speed in mm/s, range -2000..2000
        angular_mrad_s: Angular speed in 0.001 rad/s, range -6870..6870

    Returns:
        Complete protocol frame as bytes.
    """
    # Clamp values to protocol limits
    linear_x_mms = max(-2000, min(2000, linear_x_mms))
    linear_y_mms = max(-2000, min(2000, linear_y_mms))
    angular_mrad_s = max(-6870, min(6870, angular_mrad_s))

    data_length = 0x06
    cmd = CMD_MOTION
    # Pack three signed 16-bit integers big-endian
    motion_data = struct.pack('>hhh', linear_x_mms, linear_y_mms, angular_mrad_s)

    # XOR checksum: covers DataLength, CMD, and motion_data bytes
    checksum_payload = bytes([data_length, cmd]) + motion_data
    xor_check = compute_xor(checksum_payload)

    frame = bytes([FRAME_HEADER0, FRAME_HEADER1]) + checksum_payload + bytes([xor_check])
    return frame


class UartDriver:
    """
    Serial (UART) driver for the LZ_OMNI chassis.

    Opens the given serial port and provides a thread-safe method to
    send motion commands.  Incoming data is read in a background thread
    and exposed via an optional callback.
    """

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        timeout: float = 1.0,
        logger: Optional[logging.Logger] = None,
    ):
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._logger = logger or logging.getLogger(__name__)

        self._serial = None
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._rx_callback = None

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------

    def open(self) -> bool:
        """Open the serial port.  Returns True on success."""
        try:
            import serial  # lazy import so the module is loadable without pyserial
            self._serial = serial.Serial(
                port=self._port,
                baudrate=self._baudrate,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE,
                timeout=self._timeout,
            )
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._logger.info(f'UART opened: {self._port} @ {self._baudrate} baud')
            return True
        except Exception as exc:
            self._logger.error(f'Failed to open UART {self._port}: {exc}')
            return False

    def close(self) -> None:
        """Close the serial port and stop the read thread."""
        self._running = False
        if self._read_thread is not None:
            self._read_thread.join(timeout=2.0)
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
                self._logger.info('UART closed')

    def is_open(self) -> bool:
        """Return True if the serial port is currently open."""
        return self._serial is not None and self._serial.is_open

    def set_rx_callback(self, callback) -> None:
        """Register a callable(data: bytes) invoked on each received frame."""
        self._rx_callback = callback

    def send_motion(
        self,
        linear_x_mms: int,
        linear_y_mms: int,
        angular_mrad_s: int,
    ) -> bool:
        """
        Send a motion command frame to the chassis.

        Args:
            linear_x_mms: X speed in mm/s
            linear_y_mms: Y speed in mm/s
            angular_mrad_s: Angular speed in 0.001 rad/s

        Returns:
            True if the frame was written successfully.
        """
        if not self.is_open():
            self._logger.warning('UART not open – cannot send motion command')
            return False

        frame = build_motion_frame(linear_x_mms, linear_y_mms, angular_mrad_s)
        try:
            with self._lock:
                self._serial.write(frame)
            self._logger.debug(
                f'UART TX: vx={linear_x_mms} vy={linear_y_mms} w={angular_mrad_s} '
                f'frame={frame.hex()}'
            )
            return True
        except Exception as exc:
            self._logger.error(f'UART write error: {exc}')
            return False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------

    def _read_loop(self) -> None:
        """Background thread that reads incoming bytes from the chassis."""
        while self._running:
            try:
                if self._serial and self._serial.is_open and self._serial.in_waiting:
                    data = self._serial.read(self._serial.in_waiting)
                    if data and self._rx_callback:
                        self._rx_callback(data)
                else:
                    time.sleep(0.005)
            except Exception as exc:
                if self._running:
                    self._logger.error(f'UART read error: {exc}')
                time.sleep(0.1)
