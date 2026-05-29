"""Shared mission data models."""

from dataclasses import dataclass
from enum import Enum
import math
from typing import Optional


class MissionState(str, Enum):
    SEARCH_PEAK = "search_peak"
    CENTER_ON_LINE = "center_on_line"
    MEASURE_ON_LINE = "measure_on_line"
    FOLLOW_LINE = "follow_line"
    REACQUIRE = "reacquire"
    COMPLETE = "complete"
    FAILED = "failed"


@dataclass(frozen=True)
class RobotPose:
    x: float
    y: float
    z: float
    yaw: float


@dataclass(frozen=True)
class ReceiverReading:
    magnetic_x: float
    magnetic_y: float
    magnetic_z: float
    stamp_sec: float
    frame_id: str

    @property
    def signal_strength(self) -> float:
        return math.sqrt(
            self.magnetic_x * self.magnetic_x
            + self.magnetic_y * self.magnetic_y
            + self.magnetic_z * self.magnetic_z
        )


@dataclass(frozen=True)
class MoveResult:
    target: RobotPose
    reading: Optional[ReceiverReading] = None


@dataclass(frozen=True)
class MissionLogEntry:
    step: int
    state: MissionState
    pose: RobotPose
    reading: ReceiverReading
    note: str = ""
