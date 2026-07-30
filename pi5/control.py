from __future__ import annotations

from dataclasses import dataclass

import config


@dataclass
class DriveCommand:
    target_x: float
    target_y: float
    drive_time_s: float


def to_drive_command(landing_point: tuple[float, float, float], time_to_land: float) -> DriveCommand:
    target_x, target_y, _z_catch = landing_point
    if time_to_land > 0:
        drive_time = min(config.RECAL_DRIVE_TIME_S, time_to_land)
    else:
        drive_time = config.RECAL_DRIVE_TIME_S
    return DriveCommand(target_x=target_x, target_y=target_y, drive_time_s=drive_time)
