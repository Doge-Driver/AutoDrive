#! /usr/bin/python3
from math import cos, pi, radians, sin

import cv2
import numpy as np
from geometry_msgs.msg import Point32

import GlobalPath
import LaneMap
import Vehicle
from LaneMap import LaneType, findRoadPoint
from Subscribers import Lidar, VehicleStatus
from utils import getFilePath


def isStaticObstacleDetected():
    return isNearby()


def isDynamicObstacleDetected():
    return False


def isNearby():
    return sum(np.array(Lidar.ranges)[60:300] < 1.3) > 10


def isForward():
    return sum(np.array(Lidar.ranges)[165:195] < 1.3) > 10


def calcRotaionAngle():
    pass


def getEvasionPoint(rotateAngleDeg=20):
    if isNearby() == True:
        angle = pi + radians(VehicleStatus.heading)
    minRange = min(Lidar.ranges)
    x = minRange * cos(angle + radians(Lidar.ranges.index(minRange)))
    y = minRange * sin(angle + radians(Lidar.ranges.index(minRange)))
    lidar_min_x = VehicleStatus.position.x + x
    lidar_min_y = VehicleStatus.position.y + y
    if Lidar.ranges.index(minRange) + rotateAngleDeg < 360:
        rot_x = minRange * cos(
            angle + radians(Lidar.ranges.index(minRange) + rotateAngleDeg)
        )
        rot_y = minRange * sin(
            angle + radians(Lidar.ranges.index(minRange) + rotateAngleDeg)
        )
    else:
        rot_x = minRange * cos(
            angle + radians(rotateAngleDeg - (360 - Lidar.ranges.index(minRange)))
        )
        rot_y = minRange * sin(
            angle + radians(rotateAngleDeg - (360 - Lidar.ranges.index(minRange)))
        )
    lidar_min_rotate_x = VehicleStatus.position.x + rot_x
    lidar_min_rotate_y = VehicleStatus.position.y + rot_y

    return (lidar_min_rotate_x, lidar_min_rotate_y)
