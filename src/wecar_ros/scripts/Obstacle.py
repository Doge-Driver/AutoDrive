from math import cos, radians, sin

import numpy as np

from Subscribers import Lidar, VehicleStatus


def isNearby():
    return sum(np.array(Lidar.ranges)[60:300] < 1.3) > 3


def isForward():
    return sum(np.array(Lidar.ranges)[160:200] < 1.3) > 2


def getFirstEvasionPoint(rotateAngleDeg=30):
    filteredRange = Lidar.ranges[160:200]
    angle = Lidar.ANGLE_YAW + radians(VehicleStatus.heading)
    minRange = min(filteredRange)
    moreDistance = 0.5  # minRange * tan(rotateAngleDeg) ** 2
    x = minRange * cos(angle + radians(filteredRange.index(minRange)))
    y = minRange * sin(angle + radians(filteredRange.index(minRange)))

    rot_x = (moreDistance + minRange) * cos(
        angle + radians(160 + filteredRange.index(minRange) + rotateAngleDeg)
    )
    rot_y = (moreDistance + minRange) * sin(
        angle + radians(160 + filteredRange.index(minRange) + rotateAngleDeg)
    )
    lidar_min_rotate_x = VehicleStatus.position.x + rot_x
    lidar_min_rotate_y = VehicleStatus.position.y + rot_y

    return (lidar_min_rotate_x, lidar_min_rotate_y)
