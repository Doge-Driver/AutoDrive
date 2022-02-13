#! /usr/bin/python3
from enum import Enum
from math import atan2, cos, sin, sqrt
from typing import Dict, List, Tuple

import cv2
import numpy as np
from cv2 import INTER_LINEAR
from geometry_msgs.msg import Point

from utils import getFilePath


class LaneType(Enum):
    EDGE = 1
    DOT = 2
    CENTER = 3
    STOP = 4


SIM_WIDTH, SIM_HEIGHT = 40, 12
SIM_MAX_X, SIM_MAX_Y = SIM_WIDTH / 2, SIM_HEIGHT / 2
SIM_MIN_X, SIM_MIN_Y = -SIM_WIDTH / 2, -SIM_HEIGHT / 2


file = getFilePath("mapimg/labeledMap.png")
MAP = cv2.imread(file, cv2.IMREAD_GRAYSCALE)  # type: np.ndarray


IMG_HEIGHT, IMG_WIDTH = MAP.shape[:2]
IMG_MAX_X, IMG_MAX_Y = IMG_WIDTH, IMG_HEIGHT
IMG_MIN_X, IMG_MIN_Y = 0.0, 0.0
IMG_CENTER_X, IMG_CENTER_Y = int(IMG_WIDTH / 2), int(IMG_HEIGHT / 2)

SCALE_FACTOR = IMG_WIDTH / SIM_WIDTH  # IMG_WIDTH / SIM_WIDTH == IMG_HEIGHT / SIM_HEIGHT


def convertSizeSim2Img(simSize):
    return int(simSize * SCALE_FACTOR)


def convertSizeImg2Sim(imgSize):
    return imgSize / SCALE_FACTOR


def convertPointSim2Img(
    x, y
):  # type: (float, float) -> Tuple[int, int] | Tuple[None, None]
    if x > SIM_MAX_X or x < SIM_MIN_X:
        return None, None
    if y > SIM_MAX_Y or y < SIM_MIN_Y:
        return None, None
    scaledX, scaledY = x * SCALE_FACTOR, y * SCALE_FACTOR
    return IMG_CENTER_X + scaledX, IMG_CENTER_Y - scaledY


def convertPointImg2Sim(
    x, y
):  # type: (int, int) -> Tuple[float, float] | Tuple[None, None]
    if x > IMG_MAX_X or x < IMG_MIN_X:
        return None, None
    if y > IMG_MAX_Y or y < IMG_MIN_Y:
        return None, None
    scaledX, scaledY = x / SCALE_FACTOR, y / SCALE_FACTOR
    return SIM_MIN_X + scaledX, SIM_MAX_Y - scaledY


def findNearestLanePoint(simX, simY, ksize=0.5):  # SimScaled
    imgX, imgY = convertPointSim2Img(simX, simY)
    kSizeX, kSizeY = convertSizeSim2Img(ksize), convertSizeSim2Img(ksize)

    minDistance = {
        LaneType.EDGE.value: 100000,
        LaneType.DOT.value: 100000,
        LaneType.CENTER.value: 100000,
        LaneType.STOP.value: 100000,
    }  # type: dict[int, tuple[int, int]]

    nearestPoint = {
        LaneType.EDGE.value: (None, None),
        LaneType.DOT.value: (None, None),
        LaneType.CENTER.value: (None, None),
        LaneType.STOP.value: (None, None),
    }  # type: dict[int, tuple[int, int]]

    for index, value in np.ndenumerate(
        MAP[
            int(imgY - kSizeY) : int(imgY + kSizeY),
            int(imgX - kSizeX) : int(imgX + kSizeX),
        ]
    ):
        if value == 0:
            continue

        iy, ix = index[0] + imgY - kSizeY, index[1] + imgX - kSizeX
        distance = sqrt((imgX - ix) ** 2 + (imgY - iy) ** 2)

        if minDistance[value] > distance:
            minDistance[value] = convertSizeImg2Sim(distance)
            nearestPoint[value] = convertPointImg2Sim(ix, iy)

    return nearestPoint, minDistance


def findNearestLanePoints(simPoints):
    lanePoints = {
        LaneType.EDGE.value: [],
        LaneType.DOT.value: [],
        LaneType.CENTER.value: [],
        LaneType.STOP.value: [],
    }

    minDistances = {
        LaneType.EDGE.value: [],
        LaneType.DOT.value: [],
        LaneType.CENTER.value: [],
        LaneType.STOP.value: [],
    }
    for point in simPoints:
        nearestPoint, minDistance = findNearestLanePoint(point.x, point.y)

        if nearestPoint[LaneType.EDGE.value] != (None, None):
            lanePoints[LaneType.EDGE.value].append(nearestPoint[LaneType.EDGE.value])
            minDistances[LaneType.EDGE.value].append(minDistance[LaneType.EDGE.value])

        if nearestPoint[LaneType.DOT.value] != (None, None):

            lanePoints[LaneType.DOT.value].append(nearestPoint[LaneType.DOT.value])
            minDistances[LaneType.DOT.value].append(minDistance[LaneType.DOT.value])
        if nearestPoint[LaneType.CENTER.value] != (None, None):
            lanePoints[LaneType.CENTER.value].append(
                nearestPoint[LaneType.CENTER.value]
            )
            minDistances[LaneType.CENTER.value].append(
                minDistance[LaneType.CENTER.value]
            )

        if nearestPoint[LaneType.STOP.value] != (None, None):
            lanePoints[LaneType.STOP.value].append(nearestPoint[LaneType.STOP.value])
            minDistances[LaneType.STOP.value].append(minDistance[LaneType.STOP.value])

    return lanePoints, minDistances


def findRoadPoint(simPathPointX, simPathPointY, distanceFromLane=0.175):  # SimScaled
    imgPathPointX, imgPathPointY = convertPointSim2Img(simPathPointX, simPathPointY)
    imgDistanceFromLane = convertSizeSim2Img(distanceFromLane)
    nearestLanePoint, _ = findNearestLanePoint(simPathPointX, simPathPointY)

    roadPoint = {
        LaneType.EDGE.value: (None, None),
        LaneType.DOT.value: (None, None),
        LaneType.CENTER.value: (None, None),
        LaneType.STOP.value: (None, None),
    }

    for key, value in nearestLanePoint.items():
        laneX, laneY = value
        if laneX == None or laneY == None:
            continue
        imgLaneX, imgLaneY = convertPointSim2Img(laneX, laneY)
        vectorSize = sqrt(
            (imgPathPointX - imgLaneX) ** 2 + (imgPathPointY - imgLaneY) ** 2
        )

        if vectorSize == 0:
            continue

        vectorX = (imgPathPointX - imgLaneX) * imgDistanceFromLane / vectorSize
        vectorY = (imgPathPointY - imgLaneY) * imgDistanceFromLane / vectorSize
        roadPoint[key] = convertPointImg2Sim(imgLaneX + vectorX, imgLaneY + vectorY)

    return roadPoint


def findRoadPoints(simPoints, distance=0.175):  # SimScaled
    roadPoints = {
        LaneType.EDGE.value: [],
        LaneType.DOT.value: [],
        LaneType.CENTER.value: [],
        LaneType.STOP.value: [],
    }  # type: dict[int, list]
    for point in simPoints:
        x, y = point.x, point.y
        roadPoint = findRoadPoint(x, y, distance)
        if roadPoint[LaneType.EDGE.value] != (None, None):
            roadPoints[LaneType.EDGE.value].append(roadPoint[LaneType.EDGE.value])
        if roadPoint[LaneType.DOT.value] != (None, None):
            roadPoints[LaneType.DOT.value].append(roadPoint[LaneType.DOT.value])
        if roadPoint[LaneType.CENTER.value] != (None, None):
            roadPoints[LaneType.CENTER.value].append(roadPoint[LaneType.CENTER.value])
        if roadPoint[LaneType.STOP.value] != (None, None):
            roadPoints[LaneType.STOP.value].append(roadPoint[LaneType.STOP.value])

    return roadPoints


def findRoadPointJW(pathPoint, lanePoint, distance):
    pathX, pathY = pathPoint
    laneX, laneY = lanePoint
    if pathX == laneX:
        roadX = laneX
        if laneY > pathY:
            roadY = laneY - distance
        else:
            roadY = laneY + distance
    else:
        theta = atan2((pathY - laneY), (pathX - laneX))
        if laneX > pathX:
            roadX = laneX - abs(distance * cos(theta))
        else:
            roadX = laneX + abs(distance * cos(theta))
        if laneY > pathY:
            roadY = laneY - abs(distance * sin(theta))
        else:
            roadY = laneY + abs(distance * sin(theta))

    return (roadX, roadY)


def findRoadPointsJW(pathPoint, lanePoints, distance):
    roadPoints = []
    for i in range(len(lanePoints)):  # points = local_path positions
        x, y = pathPoint[i].x, pathPoint[i].y
        roadPoints.append(findRoadPointJW((x, y), lanePoints[i], distance))

    return roadPoints
