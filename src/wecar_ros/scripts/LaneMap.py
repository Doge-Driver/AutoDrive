#! /usr/bin/python3
from enum import Enum
from math import sqrt
from typing import List, Tuple

import cv2
import numpy as np

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
map = cv2.imread(file, cv2.IMREAD_GRAYSCALE)  # type: np.ndarray
__map = map

IMG_HEIGHT, IMG_WIDTH = map.shape[:2]
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
):  # type: (int, int) -> Tuple[int, int] | Tuple[None, None]
    if x > SIM_MAX_X or x < SIM_MIN_X:
        return None, None
    if y > SIM_MAX_Y or y < SIM_MIN_Y:
        return None, None
    scaledX, scaledY = x * SCALE_FACTOR, y * SCALE_FACTOR
    return int(IMG_CENTER_X + scaledX), int(IMG_CENTER_Y - scaledY)


def convertPointImg2Sim(
    x, y
):  # type: (int, int) -> Tuple[int, int] | Tuple[None, None]
    if x > IMG_MAX_X or x < IMG_MIN_X:
        return None, None
    if x > IMG_MAX_Y or x < IMG_MIN_Y:
        return None, None
    scaledX, scaledY = x / SCALE_FACTOR, y / SCALE_FACTOR
    return SIM_MIN_X + scaledX, SIM_MAX_Y - scaledY


def getMap(self):
    return __map


def findNearestLanePoint(simX, simY, ksize=0.5):  # SimScaled
    imgX, imgY = convertPointSim2Img(simX, simY)
    kSizeX, kSizeY = convertSizeSim2Img(ksize), convertSizeSim2Img(ksize)

    minDistance = {
        LaneType.EDGE.value: 10000,
        LaneType.DOT.value: 10000,
        LaneType.CENTER.value: 10000,
        LaneType.STOP.value: 10000,
    }  # type: dict[int, tuple[int, int]]

    nearestPoint = {
        LaneType.EDGE.value: (-1, -1),
        LaneType.DOT.value: (-1, -1),
        LaneType.CENTER.value: (-1, -1),
        LaneType.STOP.value: (-1, -1),
    }  # type: dict[int, tuple[int, int]]

    for index, value in np.ndenumerate(
        __map[
            imgX - kSizeX : imgX + kSizeX,
            imgY - kSizeY : imgY + kSizeY,
        ]
    ):
        # 안해도 안전할거라 믿어
        # if not value in LaneType.name:
        #     continue
        iy, ix = index
        distance = sqrt((imgX - ix) ** 2 + (imgY - iy) ** 2)

        if minDistance[value] > distance:
            minDistance[value] = convertSizeImg2Sim(distance)
            nearestPoint[value] = convertPointImg2Sim(iy, ix)

    return nearestPoint, minDistance


def findNearestLanePoints(simPoints):  # SimScaled
    lanePoints = (
        []
    )  # type: List[tuple[dict[int, tuple[int, int]], dict[int, tuple[int, int]]]]
    for point in simPoints:
        x, y = point
        lanePoints.append(findNearestLanePoint(x, y))

    return lanePoints


def findRoadPoint(simX, simY, distanceFromLane=0.15):  # SimScaled
    imgX, imgY = convertPointSim2Img(simX, simY)
    convertedDistanceFromLane = convertSizeSim2Img(distanceFromLane)
    nearestPoint, _ = findNearestLanePoint(simX, simY)

    roadPoint = {
        LaneType.EDGE.value: (-1, -1),
        LaneType.DOT.value: (-1, -1),
        LaneType.CENTER.value: (-1, -1),
        LaneType.STOP.value: (-1, -1),
    }

    for key, value in nearestPoint.items():
        simX, simY = value
        if simX == -1 or simY == -1:
            continue
        vectorSize = sqrt((imgX - simX) ** 2 + (imgY - simY) ** 2)

        vectorX = (imgX - simX) * convertedDistanceFromLane / vectorSize
        vectorY = (imgY - simY) * convertedDistanceFromLane / vectorSize
        roadPoint[key] = convertSizeImg2Sim(imgX + vectorX), convertSizeImg2Sim(
            imgY + vectorY
        )

    return roadPoint


def findRoadPoints(simPoints):  # SimScaled
    roadPoints = []  # type: List[dict[int, tuple[int, int]]]
    for point in simPoints:
        x, y = point
        roadPoints.append(findRoadPoint(x, y))

    return roadPoints
