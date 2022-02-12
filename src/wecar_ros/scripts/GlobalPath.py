from math import sqrt

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path

from Subscribers import VehicleStatus
from utils import getFilePath


def getGlobalPath():
    return __globalPath


def getSlicedGlobalPath(interestPointCount=5):
    lastPathIndex = min(__currentPathIndex + interestPointCount, len(__globalPath))
    return __globalPath.poses[__currentPathIndex:lastPathIndex]


def getCurrentPathIndex():
    return __currentPathIndex


def load(pathFileName="path/global_path.txt"):
    filePath = getFilePath(pathFileName)
    file = open(filePath, "r")

    lines = file.readlines()
    for line in lines:
        x, y, z = map(float, line.split())

        tmpPose = PoseStamped()
        tmpPose.pose.position.x = x
        tmpPose.pose.position.y = y
        tmpPose.pose.position.z = z

        __globalPath.poses.append(tmpPose)

    file.close()


def updatePathIndex():
    vehiclePosition = VehicleStatus.position
    currentDistance = __calcDistance(
        vehiclePosition,
        __globalPath.poses[__currentPathIndex].pose.position,
    )
    nextDistance = __calcDistance(
        vehiclePosition,
        __globalPath.poses[__currentPathIndex + 1].pose.position,
    )

    if currentDistance > nextDistance:
        __currentPathIndex += 1

    return __currentPathIndex


def __calcDistance(position1, position2):
    return sqrt((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2)


__currentPathIndex = 0


__globalPath = Path()
__globalPath.header.frame_id = "/map"
