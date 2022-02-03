import sys
from math import sqrt

import rospy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from rospkg import RosPack

from Subscriber import VehicleStatus


class GlobalPath:
    __pathIndex = 0

    def __init__(self, vehicleInfo: VehicleStatus, pathFileName):
        self.__vehicleInfo = vehicleInfo
        self.__load(pathFileName)

    def getPath(self):
        return self.__globalPath

    def getPathIndex(self):
        return self.__pathIndex

    def __load(self, pathFileName):
        self.__globalPath = Path()
        self.__globalPath.header.frame_id = "/map"

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
        vehiclePosition = self.__vehicleInfo.retrieve().get()
        currentDistance = self.__calcDistance(
            vehiclePosition,
            self.__globalPath.poses[self.__pathIndex].pose.position,
        )
        nextDistance = self.__calcDistance(
            vehiclePosition,
            self.__globalPath.poses[self.__pathIndex + 1].pose.position,
        )

        if currentDistance > nextDistance:
            self.__pathIndex += 1

        return self.__pathIndex

    def __calcDistance(self, position1, position2):
        return sqrt((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2)


# class PathPlanner:
#     def __init__(self, vehicleInfo: VehicleStatus, globalPathFileName):
#         self.vehicleInfo = vehicleInfo
#         self.globalPath = GlobalPath(vehicleInfo, globalPathFileName)

#     # Returns Path in list
#     def getPath(self):
#         pass


# if __name__ == "__main__":
#     rospy.init_node("test", anonymous=True)
#     globalPathFile = rospy.myargv(argv=sys.argv)[1]
#     pathPlanner = PathPlanner(globalPathFile)
