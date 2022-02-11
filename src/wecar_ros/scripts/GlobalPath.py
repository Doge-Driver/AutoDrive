import sys
from math import sqrt
from typing import List

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rospkg import RosPack

from Subscriber import VehicleStatus


class GlobalPath:
    __currentPathIndex = 0

    def __init__(self, pathFileName):
        self.__globalPath = Path()
        self.__globalPath.header.frame_id = "/map"
        self.__load(pathFileName)

    def getGlobalPath(self):
        return self.__globalPath

    def getSlicedGlobalPath(self, interestPointCount = 5):
        lastPathIndex = min(
            self.__currentPathIndex + interestPointCount, len(self.__globalPath)
        )
        return self.__globalPath.poses[self.__currentPathIndex:lastPathIndex]

    def getCurrentPathIndex(self):
        return self.__currentPathIndex

    def __load(self, pathFileName):
        packageLocation = RosPack().get_path("wecar_ros")
        file = open(f"{packageLocation}/path/{pathFileName}.txt", "r")

        lines = file.readlines()
        for line in lines:
            x, y, z = map(float, line.split())

            tmpPose = PoseStamped()
            tmpPose.pose.position.x = x
            tmpPose.pose.position.y = y
            tmpPose.pose.position.z = z

            self.__globalPath.poses.append(tmpPose)

        file.close()

    def updatePathIndex(self):
        vehiclePosition = VehicleStatus().retrieve().get().position
        currentDistance = self.__calcDistance(
            vehiclePosition,
            self.__globalPath.poses[self.__currentPathIndex].pose.position,
        )
        nextDistance = self.__calcDistance(
            vehiclePosition,
            self.__globalPath.poses[self.__currentPathIndex + 1].pose.position,
        )

        if currentDistance > nextDistance:
            self.__currentPathIndex += 1

        return self.__currentPathIndex

    def __calcDistance(self, position1, position2):
        return sqrt((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2)
