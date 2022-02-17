#! /usr/bin/python3
from math import cos, pi, radians, sin

import cv2
import numpy as np
from geometry_msgs.msg import Point32
from numpy import append

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
    return sum(np.array(Lidar.filteredRanges)[60:300] < 1.3) > 3


def isForward():
    return sum(np.array(Lidar.filteredRanges)[160:200] < 2) > 2


def calcRotaionAngle():
    pass


def getEvasionPoint(rotateAngleDeg=40):
    # if isNearby() == True:
    filteredRange = Lidar.filteredRanges
    angle = pi + radians(VehicleStatus.heading)
    minRange = min(filteredRange)
    x = minRange * cos(angle + radians(filteredRange.index(minRange)))
    y = minRange * sin(angle + radians(filteredRange.index(minRange)))
    lidar_min_x = VehicleStatus.position.x + x
    lidar_min_y = VehicleStatus.position.y + y

    print(lidar_min_x, lidar_min_y)
    if filteredRange.index(minRange) + rotateAngleDeg < 360:
        rot_x = minRange * cos(
            angle + radians(filteredRange.index(minRange) + rotateAngleDeg)
        )
        rot_y = minRange * sin(
            angle + radians(filteredRange.index(minRange) + rotateAngleDeg)
        )
    else:
        rot_x = minRange * cos(
            angle + radians(rotateAngleDeg + filteredRange.index(minRange))
        )
        rot_y = minRange * sin(
            angle + radians(rotateAngleDeg + filteredRange.index(minRange))
        )
    lidar_min_rotate_x = VehicleStatus.position.x + rot_x
    lidar_min_rotate_y = VehicleStatus.position.y + rot_y

    return (lidar_min_rotate_x, lidar_min_rotate_y)
