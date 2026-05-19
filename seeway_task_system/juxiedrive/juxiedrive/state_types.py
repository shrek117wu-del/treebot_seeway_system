"""State and configuration data types for JuxieDrive joints."""

from dataclasses import dataclass, field
from typing import Optional
import time


@dataclass
class JointConfig:
    """Static joint configuration loaded from YAML."""

    name: str
    node_id: int
    model: str = 'generic'
    can_protocol: str = 'canfd'
    reduction_ratio: float = 1.0
    min_position_deg: float = -180.0
    max_position_deg: float = 180.0
    mit_position_limit_deg: float = 180.0
    mit_velocity_limit_rpm: float = 3000.0
    mit_torque_limit_nm: float = 45.0


@dataclass
class JointRuntimeState:
    """Latest observed runtime state for one joint module."""

    joint_name: str
    node_id: int
    online: bool = False
    heartbeat_state: int = 0
    control_mode: int = 0
    enabled: bool = False
    brake_released: bool = False
    fault: bool = False
    target_reached: bool = False
    position_deg: float = 0.0
    velocity_rpm: float = 0.0
    current_ma: float = 0.0
    coil_temperature_c: float = 0.0
    mos_temperature_c: float = 0.0
    error_code: int = 0
    status_word: int = 0
    manufacturer_name: str = ''
    joint_model: str = ''
    firmware_version: str = ''
    hardware_version: str = ''
    last_feedback_time: float = field(default_factory=time.monotonic)
    last_sdo_time: float = field(default_factory=time.monotonic)
    last_heartbeat_time: float = field(default_factory=time.monotonic)
    last_raw_feedback: Optional[bytes] = None

    def touch_feedback(self, raw_data: Optional[bytes] = None) -> None:
        self.online = True
        self.last_feedback_time = time.monotonic()
        if raw_data is not None:
            self.last_raw_feedback = raw_data

    def touch_sdo(self) -> None:
        self.online = True
        self.last_sdo_time = time.monotonic()

    def touch_heartbeat(self, state: int) -> None:
        self.online = True
        self.heartbeat_state = state
        self.last_heartbeat_time = time.monotonic()

    def refresh_online(self, timeout_sec: float) -> bool:
        latest = max(self.last_feedback_time, self.last_sdo_time, self.last_heartbeat_time)
        self.online = (time.monotonic() - latest) <= timeout_sec
        return self.online
