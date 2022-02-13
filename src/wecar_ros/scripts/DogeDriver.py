#! /usr/bin/python3

from math import radians
from typing import List

import cv2
import rospy
from geometry_msgs.msg import Point32

import GlobalPath
import LaneMap
import Vehicle
from LaneMap import LaneType, findRoadPoint
from planner_test import Planner, purePursuit
from Subscribers import Lidar, VehicleStatus
from utils import getFilePath

DEBUG = True


def drawLidarOnMap():
    lidarPoints = Lidar.convert2Points(
        angleOffset=radians(VehicleStatus.heading)
    )  # type: List[Point32]

    for lidarPoint in lidarPoints:
        mapLidarPointX, mapLidarPointY = LaneMap.convertPointSim2Img(
            vehiclePoint.x + lidarPoint.x,
            vehiclePoint.y + lidarPoint.y,
        )
        if mapLidarPointX is None or mapLidarPointY is None:
            continue

        cv2.line(
            mapImg,
            (mapLidarPointX, mapLidarPointY),
            (mapLidarPointX, mapLidarPointY),
            (182, 89, 83),
            3,
        )


rospy.init_node("doge_driver", anonymous=True)

# Load Global Path
GlobalPath.load("path/global_path.txt")


pp = purePursuit()

while not rospy.is_shutdown():
    vehiclePoint = VehicleStatus.position
    x, y = LaneMap.convertPointSim2Img(vehiclePoint.x, vehiclePoint.y)
    if x is None or y is None:
        continue

    # GlobalPath
    if GlobalPath.getCurrentPathIndex() + 1 >= len(GlobalPath.getGlobalPath()):
        Vehicle.brake()
        Vehicle.steerRadian(0)
        break
    GlobalPath.updatePathIndex()
    slicedGlobalPathPoints = GlobalPath.getSlicedGlobalPath(10)

    # Lane Info
    lanePoints, distances = LaneMap.findNearestLanePoints(
        simPoints=slicedGlobalPathPoints
    )

    planner = Planner(lanePoints[LaneType.EDGE.value], lanePoints[LaneType.DOT.value])
    roadPoints = LaneMap.findRoadPoints(
        slicedGlobalPathPoints, LaneMap.convertSizeImg2Sim(15.0)
    )

    steering = pp.steering_angle(slicedGlobalPathPoints)

    # DRIVE
    Vehicle.accel(1500)
    Vehicle.steerRadian(steering)

    if DEBUG:
        # Load Colored Map for Debugging
        colorMapFile = getFilePath("mapimg/colorLabeledMap.png")
        colormap = cv2.imread(colorMapFile, cv2.IMREAD_ANYCOLOR)

        Lidar.publishPointCloud()

        # Clone Map
        mapImg = colormap.copy()

        # Vehicle
        cv2.rectangle(
            mapImg,
            (int(x) - 8, int(y) - 8),
            (int(x) + 8, int(y) + 8),
            255,
            -1,
        )

        # Lidar
        # drawLidarOnMap()

        # Draw Global Points
        for point in GlobalPath.getGlobalPath():
            tx, ty = point.x, point.y
            tx, ty = LaneMap.convertPointSim2Img(tx, ty)
            x, y = int(tx), int(ty)
            cv2.line(mapImg, (x, y), (x, y), (255, 255, 0), 5)

        for dotroadPoint in roadPoints[LaneType.DOT.value]:
            tx, ty = dotroadPoint
            if tx is None or ty is None:
                continue
            tx, ty = LaneMap.convertPointSim2Img(tx, ty)
            x, y = int(tx), int(ty)
            cv2.line(
                mapImg,
                (x, y),
                (x, y),
                (0, 255, 255),
                5,
            )

        # cv2 window
        cv2.imshow("img", mapImg)
        cv2.waitKey(1)
