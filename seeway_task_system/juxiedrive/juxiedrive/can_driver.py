#!/usr/bin/env python3
"""SocketCAN transport for JuxieDrive CAN and CAN-FD traffic."""

from __future__ import annotations

from dataclasses import dataclass
import logging
import threading
import time
from typing import Callable, List, Optional


@dataclass(frozen=True)
class CanFrame:
    """Normalized CAN frame used by the driver and protocol layers."""

    arbitration_id: int
    data: bytes
    is_extended_id: bool = False
    is_fd: bool = False
    timestamp: float = 0.0


class CanDriver:
    """Thin python-can wrapper with background receive dispatch."""

    def __init__(
        self,
        channel: str = 'can0',
        bitrate: int = 1_000_000,
        data_bitrate: int = 5_000_000,
        fd_enabled: bool = True,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._channel = channel
        self._bitrate = bitrate
        self._data_bitrate = data_bitrate
        self._fd_enabled = fd_enabled
        self._logger = logger or logging.getLogger(__name__)
        self._bus = None
        self._callbacks: List[Callable[[CanFrame], None]] = []
        self._lock = threading.Lock()
        self._running = False
        self._thread: Optional[threading.Thread] = None

    def register_callback(self, callback: Callable[[CanFrame], None]) -> None:
        self._callbacks.append(callback)

    def open(self) -> bool:
        try:
            import can

            kwargs = {
                'channel': self._channel,
                'interface': 'socketcan',
                'bitrate': self._bitrate,
            }
            if self._fd_enabled:
                kwargs['fd'] = True
                kwargs['data_bitrate'] = self._data_bitrate
            try:
                self._bus = can.Bus(**kwargs)
            except TypeError:
                kwargs.pop('interface', None)
                kwargs['bustype'] = 'socketcan'
                self._bus = can.interface.Bus(**kwargs)

            self._running = True
            self._thread = threading.Thread(target=self._read_loop, daemon=True)
            self._thread.start()
            self._logger.info(
                'CAN bus opened: %s @ %d bps%s',
                self._channel,
                self._bitrate,
                f' (FD data={self._data_bitrate})' if self._fd_enabled else '',
            )
            return True
        except Exception as exc:
            self._logger.error('Failed to open CAN channel %s: %s', self._channel, exc)
            return False

    def close(self) -> None:
        self._running = False
        if self._thread is not None:
            self._thread.join(timeout=2.0)
            self._thread = None
        with self._lock:
            if self._bus is not None:
                try:
                    self._bus.shutdown()
                except Exception:
                    pass
                self._bus = None

    def is_open(self) -> bool:
        return self._bus is not None

    def send(self, arbitration_id: int, data: bytes, *, is_fd: bool = False, is_extended_id: bool = False) -> bool:
        if self._bus is None:
            self._logger.warning('CAN bus is not open; dropping frame 0x%03X', arbitration_id)
            return False
        try:
            import can

            with self._lock:
                self._bus.send(
                    can.Message(
                        arbitration_id=arbitration_id,
                        data=data,
                        is_extended_id=is_extended_id,
                        is_fd=is_fd,
                    )
                )
            self._logger.debug(
                'CAN TX id=0x%03X fd=%s data=%s',
                arbitration_id,
                is_fd,
                data.hex(),
            )
            return True
        except Exception as exc:
            self._logger.error('CAN send error on 0x%03X: %s', arbitration_id, exc)
            return False

    def _read_loop(self) -> None:
        while self._running:
            try:
                if self._bus is None:
                    time.sleep(0.1)
                    continue
                msg = self._bus.recv(timeout=0.1)
                if msg is None:
                    continue
                frame = CanFrame(
                    arbitration_id=int(msg.arbitration_id),
                    data=bytes(msg.data),
                    is_extended_id=bool(msg.is_extended_id),
                    is_fd=bool(getattr(msg, 'is_fd', False)),
                    timestamp=float(getattr(msg, 'timestamp', time.time())),
                )
                self._logger.debug('CAN RX id=0x%03X fd=%s data=%s', frame.arbitration_id, frame.is_fd, frame.data.hex())
                for callback in list(self._callbacks):
                    callback(frame)
            except Exception as exc:
                if self._running:
                    self._logger.error('CAN receive error: %s', exc)
                time.sleep(0.1)
