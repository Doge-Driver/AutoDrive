#! /usr/bin/python3
from enum import Enum
from math import sqrt
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
IMG_MIN_X, IMG_MIN_Y = 0, 0
IMG_CENTER_X, IMG_CENTER_Y = int(IMG_WIDTH / 2), int(IMG_HEIGHT / 2)

SCALE_FACTOR = IMG_WIDTH / SIM_WIDTH  # IMG_WIDTH / SIM_WIDTH == IMG_HEIGHT / SIM_HEIGHT


def convertSizeSim2Img(simSize):
    return int(simSize * SCALE_FACTOR)


def convertSizeImg2Sim(imgSize):
    return imgSize / SCALE_FACTOR


def convertPointSim2Img(
    x, y
):  # type: (float, float) -> Tuple[float, float] | Tuple[None, None]
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
            imgY - kSizeY : imgY + kSizeY,
            imgX - kSizeX : imgX + kSizeX,
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

        lanePoints[LaneType.EDGE.value].append(nearestPoint[LaneType.EDGE.value])
        lanePoints[LaneType.DOT.value].append(nearestPoint[LaneType.DOT.value])
        lanePoints[LaneType.CENTER.value].append(nearestPoint[LaneType.CENTER.value])
        lanePoints[LaneType.STOP.value].append(nearestPoint[LaneType.STOP.value])

        minDistances[LaneType.EDGE.value].append(minDistance[LaneType.EDGE.value])
        minDistances[LaneType.DOT.value].append(minDistance[LaneType.DOT.value])
        minDistances[LaneType.CENTER.value].append(minDistance[LaneType.CENTER.value])
        minDistances[LaneType.STOP.value].append(minDistance[LaneType.STOP.value])

    return lanePoints, minDistances


def findRoadPoint(simPathPointX, simPathPointY, distanceFromLane=0.15):  # SimScaled
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

        vectorX = (imgPathPointX - imgLaneX) * imgDistanceFromLane / vectorSize
        vectorY = (imgPathPointY - imgLaneY) * imgDistanceFromLane / vectorSize
        roadPoint[key] = convertPointImg2Sim(imgLaneX + vectorX, imgLaneY + vectorY)

    return roadPoint


def findRoadPoints(simPoints):  # SimScaled
    roadPoints = []  # type: List[dict[int, tuple[int, int]]]
    for point in simPoints:
        x, y = point.x, point.y
        roadPoint = findRoadPoint(x, y)
        roadPoints.append(roadPoint)

    return roadPoints
