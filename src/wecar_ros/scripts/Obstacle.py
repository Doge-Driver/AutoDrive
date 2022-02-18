from math import cos, radians, sin

import numpy as np

from Subscribers import Lidar, VehicleStatus


def isStaticObstacle():
    return isNearby()


def isNearby():
    return sum(np.array(Lidar.ranges)[60:300] < 1.3) > 3


def isForward():
    return sum(np.array(Lidar.ranges)[160:200] < 2) > 2


def calcRotaionAngle():
    pass


def getEvasionPoint(rotateAngleDeg=40):
    filteredRange = Lidar.ranges
    angle = Lidar.ANGLE_YAW + radians(VehicleStatus.heading)
    minRange = min(filteredRange)
    moreDistance = 1.0
    x = minRange * cos(angle + radians(filteredRange.index(minRange)))
    y = minRange * sin(angle + radians(filteredRange.index(minRange)))
    lidar_min_x = VehicleStatus.position.x + x
    lidar_min_y = VehicleStatus.position.y + y

    print(lidar_min_x, lidar_min_y)
    rot_x = (moreDistance + minRange) * cos(
        angle + radians(filteredRange.index(minRange) + rotateAngleDeg)
    )
    rot_y = (moreDistance + minRange) * sin(
        angle + radians(filteredRange.index(minRange) + rotateAngleDeg)
    )
    lidar_min_rotate_x = VehicleStatus.position.x + rot_x
    lidar_min_rotate_y = VehicleStatus.position.y + rot_y

    return (lidar_min_rotate_x, lidar_min_rotate_y)
