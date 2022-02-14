#! /usr/bin/python3

from math import radians
from time import time
from typing import List

import cv2
import rospy
from geometry_msgs.msg import Point32

import GlobalPath
import LaneMap
import Vehicle
from LaneMap import LaneType, findRoadPoint
from Planner import Cruise
from Subscribers import Lidar, VehicleStatus
from utils import getFilePath

DEBUG = True


rospy.init_node("doge_driver", anonymous=True)

# Load Global Path
GlobalPath.load("path/global_path_old.txt")

while not rospy.is_shutdown():
    # GlobalPath
    if GlobalPath.isLastPathIndex():
        Vehicle.brake()
        Vehicle.steerRadian(0)
        break
    GlobalPath.updatePathIndex()
    slicedGlobalPathPoints = GlobalPath.getSlicedGlobalPath(10)

    # Lane Info
    lanePoints, distances = LaneMap.findNearestLanePoints(
        simPoints=slicedGlobalPathPoints
    )

    # Destinated Road Points
    roadPoints = LaneMap.findRoadPoints(
        slicedGlobalPathPoints, LaneMap.convertSizeImg2Sim(20.0)
    )

    steering = Cruise.steering(roadPoints[LaneType.DOT.value])
    # velocity = Cruise.velocity(slicedGlobalPathPoints)

    # DRIVE
    Vehicle.accel(1000)
    Vehicle.steerRadian(steering)

    if DEBUG:

        def drawPoint(mapImg, simX, simY, color, thickness=3):
            if (simX, simY) == (None, None):
                return
            x, y = LaneMap.convertPointSim2Img(simX, simY)
            if (x, y) == (None, None):
                return
            x, y = int(x), int(y)
            cv2.line(mapImg, (x, y), (x, y), color, thickness)

        def drawLidarOnMap(mapImg):
            lidarPoints = Lidar.convert2Points(
                angleOffset=radians(VehicleStatus.heading)
            )  # type: List[Point32]

            for point in lidarPoints:
                lidarPointX, lidarPointY = (
                    VehicleStatus.position.x + point.x,
                    VehicleStatus.position.y + point.y,
                )

                drawPoint(mapImg, lidarPointX, lidarPointY, (182, 89, 83), 3)

        # Load Colored Map for Debugging
        colorMapFile = getFilePath("mapimg/colorLabeledMap.png")
        colormap = cv2.imread(colorMapFile, cv2.IMREAD_ANYCOLOR)

        Lidar.publishPointCloud()

        # Clone Map
        mapImg = colormap.copy()

        # Vehicle
        vehiclePoint = VehicleStatus.position
        drawPoint(mapImg, vehiclePoint.x, vehiclePoint.y, (255, 0, 0), 15)

        # Lidar
        drawLidarOnMap(mapImg)

        # Draw Global Points
        for point in GlobalPath.getGlobalPath():
            drawPoint(mapImg, point.x, point.y, (255, 255, 0), 5)

        # Draw DOT lane Points
        for point in roadPoints[LaneType.DOT.value]:
            drawPoint(mapImg, point[0], point[1], (0, 0, 255), 5)

        # cv2 window
        cv2.imshow("img", mapImg)
        cv2.waitKey(1)
