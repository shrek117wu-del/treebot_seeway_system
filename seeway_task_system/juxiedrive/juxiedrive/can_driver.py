#!/usr/bin/env python3
"""SocketCAN transport for JuxieDrive actuators."""

from __future__ import annotations

import logging
import inspect
import time
import threading
from typing import Callable, Optional, Sequence

from .protocol import (
    CanFrame,
    CustomFeedback,
    HeartbeatInfo,
    SdoResponse,
    Tpdo1Feedback,
    CUSTOM_FEEDBACK_BASE,
    HEARTBEAT_BASE,
    SDO_RX_BASE,
    TPDO1_BASE,
    build_canopen_pdo_mapping_sequence,
    build_controlword_sequence,
    build_current_sequence,
    build_custom_command,
    build_custom_multi_command,
    build_enable_sequence,
    build_mit_command,
    build_nmt_start_frame,
    build_profile_position_sequence,
    build_profile_velocity_sequence,
    build_query_state_sequence,
    build_query_version_sequence,
    build_rpdo1_frame,
    build_sdo_read_request,
    build_sdo_write_request,
    build_sync_frame,
    parse_custom_feedback,
    parse_heartbeat,
    parse_sdo_response,
    parse_tpdo1_feedback,
)


class CanDriver:
    """SocketCAN driver that sends and parses CANopen/CAN FD frames."""

    def __init__(
        self,
        channel: str = 'can0',
        bitrate: int = 1000000,
        logger: Optional[logging.Logger] = None,
    ) -> None:
        self._channel = channel
        self._bitrate = bitrate
        self._logger = logger or logging.getLogger(__name__)
        self._bus = None
        self._lock = threading.Lock()
        self._running = False
        self._read_thread: Optional[threading.Thread] = None
        self._sdo_callback: Optional[Callable[[SdoResponse], None]] = None
        self._heartbeat_callback: Optional[Callable[[HeartbeatInfo], None]] = None
        self._custom_feedback_callback: Optional[Callable[[CustomFeedback], None]] = None
        self._tpdo1_callback: Optional[Callable[[Tpdo1Feedback], None]] = None

    def open(self) -> bool:
        """Open the CAN interface and start the receive loop."""
        try:
            import can

            bus_kwargs = {
                'channel': self._channel,
                'bustype': 'socketcan',
                'bitrate': self._bitrate,
            }
            bus_signature = inspect.signature(can.interface.Bus)
            if 'fd' in bus_signature.parameters:
                bus_kwargs['fd'] = True
            self._bus = can.interface.Bus(**bus_kwargs)
            self._running = True
            self._read_thread = threading.Thread(target=self._read_loop, daemon=True)
            self._read_thread.start()
            self._logger.info(f'CAN bus opened: {self._channel} @ {self._bitrate} bps')
            return True
        except Exception as exc:
            self._logger.error(f'Failed to open CAN bus {self._channel}: {exc}')
            return False

    def close(self) -> None:
        """Stop background receive thread and close CAN bus."""
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

    def is_open(self) -> bool:
        """Return whether the CAN interface is available."""
        return self._bus is not None

    def set_sdo_callback(self, callback: Optional[Callable[[SdoResponse], None]]) -> None:
        self._sdo_callback = callback

    def set_heartbeat_callback(self, callback: Optional[Callable[[HeartbeatInfo], None]]) -> None:
        self._heartbeat_callback = callback

    def set_custom_feedback_callback(self, callback: Optional[Callable[[CustomFeedback], None]]) -> None:
        self._custom_feedback_callback = callback

    def set_tpdo1_callback(self, callback: Optional[Callable[[Tpdo1Feedback], None]]) -> None:
        self._tpdo1_callback = callback

    def send_frame(self, arbitration_id: int, data: bytes, is_fd: bool = False) -> bool:
        """Send a single raw CAN or CAN FD message."""
        if not self.is_open():
            self._logger.warning('CAN bus is not open')
            return False
        try:
            import can

            message = can.Message(
                arbitration_id=arbitration_id,
                data=data,
                is_extended_id=False,
                is_fd=is_fd,
                bitrate_switch=is_fd,
            )
            with self._lock:
                self._bus.send(message)
            self._logger.debug(
                f'CAN TX id=0x{arbitration_id:03X} is_fd={is_fd} data={bytes(data).hex()}'
            )
            return True
        except Exception as exc:
            self._logger.error(f'CAN send error: {exc}')
            return False

    def send_sequence(self, frames: Sequence[CanFrame], interframe_delay: float = 0.01) -> bool:
        """Send a sequence of frames in order with optional delay."""
        success = True
        for index, (arbitration_id, data, is_fd) in enumerate(frames):
            success = self.send_frame(arbitration_id, data, is_fd) and success
            if interframe_delay > 0.0 and index != len(frames) - 1:
                time.sleep(interframe_delay)
        return success

    def send_nmt_start(self, node_id: int = 0x00) -> bool:
        frame = build_nmt_start_frame(node_id)
        return self.send_frame(*frame)

    def send_sync(self) -> bool:
        frame = build_sync_frame()
        return self.send_frame(*frame)

    def send_sdo_read(self, node_id: int, index: int, subindex: int = 0) -> bool:
        frame = build_sdo_read_request(node_id, index, subindex)
        return self.send_frame(*frame)

    def send_sdo_write(
        self,
        node_id: int,
        index: int,
        subindex: int,
        value: int,
        size: int,
        signed: bool = False,
    ) -> bool:
        frame = build_sdo_write_request(node_id, index, subindex, value, size, signed=signed)
        return self.send_frame(*frame)

    def send_enable_sequence(self, node_id: int) -> bool:
        return self.send_sequence(build_enable_sequence(node_id))

    def send_profile_position(
        self,
        node_id: int,
        target_position: int,
        profile_velocity: int,
        acceleration: int,
        deceleration: int,
        start_motion: bool = True,
    ) -> bool:
        return self.send_sequence(
            build_profile_position_sequence(
                node_id,
                target_position,
                profile_velocity,
                acceleration,
                deceleration,
                start_motion=start_motion,
            )
        )

    def send_profile_velocity(
        self,
        node_id: int,
        target_velocity_rpm: int,
        acceleration: int,
        deceleration: int,
    ) -> bool:
        return self.send_sequence(
            build_profile_velocity_sequence(node_id, target_velocity_rpm, acceleration, deceleration)
        )

    def send_current(self, node_id: int, target_current_ma: int) -> bool:
        return self.send_sequence(build_current_sequence(node_id, target_current_ma))

    def send_query_version(self, node_id: int) -> bool:
        return self.send_sequence(build_query_version_sequence(node_id))

    def send_query_state(self, node_id: int) -> bool:
        return self.send_sequence(build_query_state_sequence(node_id))

    def configure_canopen_pdo(self, node_id: int) -> bool:
        return self.send_sequence(build_canopen_pdo_mapping_sequence(node_id))

    def send_rpdo1(self, node_id: int, controlword: int, target_current_ma: int, target_position: int) -> bool:
        frame = build_rpdo1_frame(node_id, controlword, target_current_ma, target_position)
        return self.send_frame(*frame)

    def send_custom_command(self, *args, **kwargs) -> bool:
        frame = build_custom_command(*args, **kwargs)
        return self.send_frame(*frame)

    def send_custom_multi_command(self, commands) -> bool:
        frame = build_custom_multi_command(commands)
        return self.send_frame(*frame)

    def send_mit_command(self, *args, **kwargs) -> bool:
        frame = build_mit_command(*args, **kwargs)
        return self.send_frame(*frame)

    def _read_loop(self) -> None:
        """Receive frames and route them to the relevant parser callback."""
        while self._running:
            try:
                if self._bus is None:
                    time.sleep(0.1)
                    continue
                msg = self._bus.recv(timeout=0.1)
                if msg is None:
                    continue
                arbitration_id = int(msg.arbitration_id)
                data = bytes(msg.data)
                self._logger.debug(
                    f'CAN RX id=0x{arbitration_id:03X} is_fd={getattr(msg, "is_fd", False)} data={data.hex()}'
                )
                self._dispatch(arbitration_id, data)
            except Exception as exc:
                if self._running:
                    self._logger.error(f'CAN read error: {exc}')
                time.sleep(0.1)

    def _dispatch(self, arbitration_id: int, data: bytes) -> None:
        """Parse a received frame according to its documented CAN ID range."""
        try:
            if SDO_RX_BASE <= arbitration_id < SDO_RX_BASE + 0x80:
                if self._sdo_callback is not None:
                    self._sdo_callback(parse_sdo_response(arbitration_id, data))
                return
            if HEARTBEAT_BASE <= arbitration_id < HEARTBEAT_BASE + 0x80:
                if self._heartbeat_callback is not None:
                    self._heartbeat_callback(parse_heartbeat(arbitration_id, data))
                return
            if CUSTOM_FEEDBACK_BASE <= arbitration_id < CUSTOM_FEEDBACK_BASE + 0x80:
                if self._custom_feedback_callback is not None:
                    self._custom_feedback_callback(parse_custom_feedback(arbitration_id, data))
                return
            if TPDO1_BASE <= arbitration_id < TPDO1_BASE + 0x80 and len(data) >= 8:
                if self._tpdo1_callback is not None:
                    self._tpdo1_callback(parse_tpdo1_feedback(arbitration_id, data))
        except Exception as exc:
            self._logger.error(f'Failed to parse CAN RX id=0x{arbitration_id:03X}: {exc}')
