#! /usr/bin/python3
from enum import Enum
from math import sqrt
from typing import Tuple

import cv2
import numpy as np

import Vehicle
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


def inSimRange(x, y):
    if x == None or y == None:
        return False
    return SIM_MIN_X <= x <= SIM_MAX_X and SIM_MIN_Y <= y <= SIM_MAX_Y


def inImgRange(x, y):
    if x == None or y == None:
        return False
    return IMG_MIN_X <= x < IMG_MAX_X and IMG_MIN_Y <= y < IMG_MAX_Y


def safeMapAccess(x, y):
    x, y = int(x), int(y)
    if not inImgRange(x, y):
        return 0
    return MAP[y][x]


def convertSizeSim2Img(simSize):
    return simSize * SCALE_FACTOR


def convertSizeImg2Sim(imgSize):
    return imgSize / SCALE_FACTOR


def convertPointSim2Img(
    x, y
):  # type: (float, float) -> Tuple[float, float] | Tuple[None, None]
    if not inSimRange(x, y):
        return None, None
    scaledX, scaledY = x * SCALE_FACTOR, y * SCALE_FACTOR
    return IMG_CENTER_X + scaledX, IMG_CENTER_Y - scaledY


def convertPointImg2Sim(
    x, y
):  # type: (float, float) -> Tuple[float, float] | Tuple[None, None]
    if not inImgRange(x, y):
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
    }  # type: dict[float, float]

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


intersectionPoints = [convertPointImg2Sim(2063, 462)]


def isIntersection():
    for point in intersectionPoints:
        if Vehicle.distanceWith(point) < 2.5:
            return True
    return False


def findRoadPoints(slicedGlobalPath, potential):
    imgGlobal_path = []
    for point in slicedGlobalPath:
        imgGlobal_path.append(convertPointSim2Img(point.x, point.y))
    road_path = []
    px, py = imgGlobal_path[0][0], imgGlobal_path[0][1]
    for point in imgGlobal_path:
        rx, ry = findRoadPoint(
            point[0], point[1], potential
        )  # rx,ry is current roadPoint, px,py is previous roadpoint
        road_path.append(
            findRoadPoint(px + (rx - px) / 2, py + (ry - py) / 2, potential)
        )
        road_path.append(
            findRoadPoint(px + 3 * (rx - px) / 4, py + 3 * (ry - py) / 4, potential)
        )
        road_path.append(
            findRoadPoint(px + (rx - px) / 4, py + (ry - py) / 4, potential)
        )
        px, py = rx, ry
        road_path.append([rx, ry])

    sim_boRoadPoints = []
    for point in road_path:
        tx, ty = point
        tx, ty = convertPointImg2Sim(tx, ty)
        sim_boRoadPoints.append((tx, ty))

    return sim_boRoadPoints


def findRoadPoint(pathX, pathY, potential):
    float_px = pathX - int(pathX)
    float_py = pathY - int(pathY)
    pathX = int(pathX)
    pathY = int(pathY)

    premin = 0
    minn = 1

    while premin != minn:
        premin = minn
        minindex = np.argmin(potential[pathY - 1 : pathY + 2, pathX - 1 : pathX + 2])
        minn = potential[pathY - 1 + int(minindex / 3)][pathX - 1 + minindex % 3]
        pathY, pathX = int(pathY - 1 + minindex / 3), int(pathX - 1 + minindex % 3)
    return (pathX + float_px, pathY + float_py)
