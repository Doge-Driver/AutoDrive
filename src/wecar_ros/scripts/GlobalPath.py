from typing import List

from geometry_msgs.msg import Point

import Vehicle
from utils import getFilePath

__currentPathIndex = 0
__globalPathPoints = []  # type: List[Point]


def isLastPathIndex():
    return getCurrentPathIndex() + 1 >= len(__globalPathPoints)


def getGlobalPath():  # type: () -> List[Point]
    return __globalPathPoints


def getSlicedGlobalPath(interestPointCount=5):  # type: (int) -> List[Point]
    lastPathIndex = min(
        __currentPathIndex + interestPointCount, len(__globalPathPoints)
    )
    return __globalPathPoints[__currentPathIndex:lastPathIndex]


def getCurrentPathIndex():
    return __currentPathIndex


def load(pathFileName="path/global_path.txt"):
    filePath = getFilePath(pathFileName)
    file = open(filePath, "r")

    lines = file.readlines()
    for line in lines:
        x, y, z = map(float, line.split())
        __globalPathPoints.append(Point(x, y, z))

    file.close()


def updatePathIndex():
    global __currentPathIndex
    currentDistance = Vehicle.distanceWith(__globalPathPoints[__currentPathIndex])
    nextDistance = Vehicle.distanceWith(__globalPathPoints[__currentPathIndex + 1])

    if currentDistance > nextDistance:
        __currentPathIndex += 1

    return __currentPathIndex
