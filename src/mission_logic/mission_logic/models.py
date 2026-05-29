"""Shared mission data models."""

from dataclasses import dataclass
from typing import Optional


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


@dataclass(frozen=True)
class MoveResult:
    target: RobotPose
    reading: Optional[ReceiverReading] = None
