#! /usr/bin/python3

from math import radians
from typing import List

import cv2
import rospy
from geometry_msgs.msg import Point32
from numpy import True_

import GlobalPath
import LaneMap
import Vehicle
from LaneMap import LaneType
from planner_test import Planner, purePursuit
from Subscribers import Lidar, VehicleStatus
from utils import getFilePath

DEBUG = True


def drawPointOnMap(x, y):
    pass


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
            255,
            3,
        )


rospy.init_node("doge_driver", anonymous=True)

# Load Global Path
GlobalPath.load("path/global_path.txt")

# Load Colored Map for Debugging
colorMapFile = getFilePath("mapimg/colorLabeledMap.png")
colormap = cv2.imread(colorMapFile, cv2.IMREAD_ANYCOLOR)

pp = purePursuit()

while not rospy.is_shutdown():
    vehiclePoint = VehicleStatus.position
    x, y = LaneMap.convertPointSim2Img(vehiclePoint.x, vehiclePoint.y)
    if x is None or y is None:
        continue

    # GlobalPath
    GlobalPath.updatePathIndex()
    slicedGlobalPathPoints = GlobalPath.getSlicedGlobalPath(100000)

    # Lane Info
    lanePoints, distances = LaneMap.findNearestLanePoints(
        simPoints=slicedGlobalPathPoints
    )

    planner = Planner(lanePoints[LaneType.EDGE.value], lanePoints[LaneType.DOT.value])

    mapSlicedGlobalPathPoints = []
    for point in slicedGlobalPathPoints:
        mapSlicedGlobalPathPoints.append(LaneMap.convertPointSim2Img(point.x, point.y))

    # TODO: use findNearestLanePoint from LaneMap module
    imgRoadPoints = planner.calcImgRoadPoints(
        mapSlicedGlobalPathPoints, lanePoints[LaneType.DOT.value], 2
    )

    simRoadPoints = []
    for point in imgRoadPoints:
        tx, ty = point
        simRoadPoints.append(LaneMap.convertPointImg2Sim(tx, ty))

    # Set Steering
    pp.getPath(simRoadPoints)
    steering = pp.steering_angle()

    # DRIVE
    Vehicle.accel(1000)
    Vehicle.steerRadian(steering)

    if DEBUG:
        Lidar.publishPointCloud()

        # Clone Map
        mapImg = colormap.copy()

        # Vehicle
        cv2.rectangle(
            mapImg,
            (x - 8, y - 8),
            (x + 8, y + 8),
            255,
            -1,
        )

        # Lidar
        # drawLidarOnMap()

        # Draw Global Points
        for point in GlobalPath.getGlobalPath():
            tx, ty = point.x, point.y
            x, y = LaneMap.convertPointSim2Img(tx, ty)
            cv2.line(mapImg, (x, y), (x, y), (255, 255, 0), 5)

        # Draw local Points
        for point in imgRoadPoints:
            cv2.line(
                mapImg,
                (int(point[0]), int(point[1])),
                (int(point[0]), int(point[1])),
                (0, 255, 255),
                5,
            )

        # cv2 window
        cv2.imshow("img", mapImg)
        cv2.waitKey(1)
