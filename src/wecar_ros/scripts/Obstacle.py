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
    return sum(np.array(filterRanges())[60:300] < 1.3) > 3


def isForward():
    return sum(np.array(filterRanges())[160:200] < 2) > 2


def calcRotaionAngle():
    pass


def filterRanges():
    # global filteredRange
    angle = 180 + radians(VehicleStatus.heading)
    filteredRange = []

    for range in Lidar.ranges:
        if range > 10.0:
            filteredRange.append(1000)
            continue
        x = range * cos(angle)  # + vehicleStatus.position.x,
        y = range * sin(angle)  # + vehicleStatus.position.y,
        angle += Lidar.angleIncrement
        lidarPointX, lidarPointY = (
            VehicleStatus.position.x + x,
            VehicleStatus.position.y + y,
        )
        if (
            lidarPointX > LaneMap.SIM_MAX_X
            or lidarPointY > LaneMap.SIM_MAX_Y
            or lidarPointX < LaneMap.SIM_MIN_X
            or lidarPointY < LaneMap.SIM_MIN_Y
        ):
            filteredRange.append(1000)
        else:
            filteredRange.append(range)

    return filteredRange


def getEvasionPoint(rotateAngleDeg=40):
    # if isNearby() == True:
    filteredRange = filterRanges()
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
