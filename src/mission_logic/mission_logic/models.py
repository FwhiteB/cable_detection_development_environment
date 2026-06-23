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
    signal_strength: float
    depth: float
    current: float
    pipeline_heading_degrees: float
    signal_strength_percent: float
    left_arrow: bool
    right_arrow: bool
    stamp_sec: float
    frame_id: str



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
