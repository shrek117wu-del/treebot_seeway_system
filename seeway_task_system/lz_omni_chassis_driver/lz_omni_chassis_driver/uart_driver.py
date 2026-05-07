#!/usr/bin/env python3
"""UART driver for the LZ_OMNI omnidirectional chassis."""

import logging
import threading
import time
from typing import Callable, Optional

from .protocol import (
    AuxInfo,
    BatteryInfo,
    ChassisStatus,
    FrameParser,
    VersionInfo,
    build_motor_control_frame,
    build_omni_control_frame,
    build_version_query_frame,
    CMD_AUX_INFO,
    CMD_BATTERY_INFO,
    CMD_CHASSIS_STATUS,
    CMD_VERSION_QUERY,
    FRAME_HEADER0,
    FRAME_HEADER1,
    compute_xor,
)

CMD_MOTION = 0x02


def build_motion_frame(linear_x_mms: int, linear_y_mms: int, angular_mrad_s: int) -> bytes:
    """Build legacy CMD 0x02 motion frame kept for backward compatibility tests."""
    vx = max(-2000, min(2000, linear_x_mms))
    vy = max(-2000, min(2000, linear_y_mms))
    vw = max(-6870, min(6870, angular_mrad_s))
    motion_data = int(vx).to_bytes(2, 'big', signed=True)
    motion_data += int(vy).to_bytes(2, 'big', signed=True)
    motion_data += int(vw).to_bytes(2, 'big', signed=True)
    checksum_payload = bytes([0x06, CMD_MOTION]) + motion_data
    return bytes([FRAME_HEADER0, FRAME_HEADER1]) + checksum_payload + bytes([compute_xor(checksum_payload)])


class UartDriver:
    """Serial communication driver implementing the LZ_OMNI UART protocol."""

    def __init__(
        self,
        port: str = '/dev/ttyUSB0',
        baudrate: int = 115200,
        timeout: float = 1.0,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        """Initialise UART parameters and callbacks."""
        self._port = port
        self._baudrate = baudrate
        self._timeout = timeout
        self._logger = logger or logging.getLogger(__name__)

        self._serial = None
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._parser = FrameParser()

        self._battery_info_callback: Optional[Callable[[BatteryInfo], None]] = None
        self._chassis_status_callback: Optional[Callable[[ChassisStatus], None]] = None
        self._aux_info_callback: Optional[Callable[[AuxInfo], None]] = None
        self._version_info_callback: Optional[Callable[[VersionInfo], None]] = None

    def open(self) -> bool:
        """Open the serial port and start the read loop thread."""
        try:
            import serial

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
        """Close the serial port and stop the read loop thread."""
        self._running = False
        if self._read_thread is not None:
            self._read_thread.join(timeout=2.0)
        with self._lock:
            if self._serial and self._serial.is_open:
                self._serial.close()
                self._logger.info('UART closed')

    def is_open(self) -> bool:
        """Return ``True`` when serial link is currently open."""
        return self._serial is not None and self._serial.is_open

    def send_motion(self, linear_x_mms: int, linear_y_mms: int, angular_mrad_s: int) -> bool:
        """Send CMD 0x40 omnidirectional chassis speed control."""
        frame = build_omni_control_frame(linear_x_mms, linear_y_mms, angular_mrad_s)
        return self._send_frame(
            frame,
            f'UART TX CMD=0x40 vx={linear_x_mms} vy={linear_y_mms} vw={angular_mrad_s}',
        )

    def send_motor_control(self, m1: int, m2: int, m3: int, m4: int) -> bool:
        """Send CMD 0x01 wheel RPM control command."""
        frame = build_motor_control_frame(m1, m2, m3, m4)
        return self._send_frame(frame, f'UART TX CMD=0x01 m=[{m1},{m2},{m3},{m4}]')

    def send_version_query(self) -> bool:
        """Send CMD 0x11 firmware version query command."""
        frame = build_version_query_frame()
        return self._send_frame(frame, 'UART TX CMD=0x11 version query')

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

    def _send_frame(self, frame: bytes, debug_message: str) -> bool:
        """Write raw protocol frame to UART with thread safety and logging."""
        if not self.is_open():
            self._logger.warning('UART not open – cannot send command')
            return False
        try:
            with self._lock:
                self._serial.write(frame)
            self._logger.debug(f'{debug_message} frame={frame.hex()}')
            return True
        except Exception as exc:
            self._logger.error(f'UART write error: {exc}')
            return False

    def _read_loop(self) -> None:
        """Read incoming bytes, parse framed payloads, and dispatch callbacks."""
        while self._running:
            try:
                if self._serial and self._serial.is_open and self._serial.in_waiting:
                    raw = self._serial.read(self._serial.in_waiting)
                    if not raw:
                        continue
                    self._logger.debug(f'UART RX raw={raw.hex()}')
                    for cmd, payload in self._parser.feed(raw):
                        self._logger.debug(f'UART RX frame cmd=0x{cmd:02X} payload={payload.hex()}')
                        self._dispatch(cmd, payload)
                else:
                    time.sleep(0.005)
            except Exception as exc:
                if self._running:
                    self._logger.error(f'UART read error: {exc}')
                time.sleep(0.1)

    def _dispatch(self, cmd: int, payload: bytes) -> None:
        """Parse payload by command ID and invoke matching callback."""
        try:
            if cmd == CMD_BATTERY_INFO:
                battery = BatteryInfo.from_bytes(payload)
                if self._battery_info_callback:
                    self._battery_info_callback(battery)
            elif cmd == CMD_CHASSIS_STATUS:
                chassis = ChassisStatus.from_bytes(payload)
                if self._chassis_status_callback:
                    self._chassis_status_callback(chassis)
            elif cmd == CMD_AUX_INFO:
                aux = AuxInfo.from_bytes(payload)
                if self._aux_info_callback:
                    self._aux_info_callback(aux)
            elif cmd == CMD_VERSION_QUERY:
                version = VersionInfo.from_bytes(payload)
                if self._version_info_callback:
                    self._version_info_callback(version)
            else:
                self._logger.debug(f'UART RX ignored unknown cmd=0x{cmd:02X}')
        except Exception as exc:
            self._logger.error(f'UART RX parse/dispatch error cmd=0x{cmd:02X}: {exc}')
