#! /usr/bin/python3

import time
from math import radians
from typing import List

import cv2
import rospy
from geometry_msgs.msg import Point32

import GlobalPath
import LaneMap
import Obstacle
import Vehicle
from LaneMap import LaneType, findRoadPoint
from Planner import Cruise
from Subscribers import Lidar, TrafficLight, VehicleStatus
from utils import getFilePath

DEBUG = True

if DEBUG:
    # Load Colored Map for Debugging
    colorMapFile = getFilePath("mapimg/colorLabeledMap.png")
    colormap = cv2.imread(colorMapFile, cv2.IMREAD_ANYCOLOR)
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


rospy.init_node("doge_driver", anonymous=True)
rospy.on_shutdown(Vehicle.stop)

# Load Global Path
GlobalPath.load("path/global_path4.txt")

doneMission1 = False

potentialFile = getFilePath("mapimg/Potential_v3.PNG")
potential = cv2.imread(potentialFile, cv2.IMREAD_UNCHANGED)

while not rospy.is_shutdown():
    # GlobalPath
    if GlobalPath.isLastPathIndex():
        Vehicle.stop()
        rospy.signal_shutdown("GOAL")
        break
    GlobalPath.updatePathIndex()
    slicedGlobalPathPoints = GlobalPath.getSlicedGlobalPath(10)

    # Update Lane Info Nearby Vehicle
    Vehicle.updateNearbyLanes()

    # MISSION1: Wait for 5 sec at second stop line
    if (
        not doneMission1
        and Vehicle.distanceWith(LaneMap.convertPointImg2Sim(3024, 580)) < 0.7
        # and Vehicle.isOnStopLine() # 차가 돌아가있으면 부정확함..
    ):
        doneMission1 = True
        Vehicle.brake()
        time.sleep(5)

    # MISSION2: Wait for Rotary vehicles

    # MISSION3: Traffic Light
    if LaneMap.isIntersection() and Vehicle.isOnStopLine():
        while not TrafficLight.isLeftGreen():
            Vehicle.brake()

    # MISSION4: Avoid Static Obstacles
    while Obstacle.isStaticObstacleDetected():
        evasionPoint = Obstacle.getEvasionPoint()
        drawPoint()
        while Vehicle.distanceWith(evasionPoint) < 0.2:
            Cruise.velocity([evasionPoint])
            Cruise.steering([evasionPoint])

    # MISSION5: Emergency Brake when Dynamic Obstacles
    while Obstacle.isDynamicObstacleDetected():
        Vehicle.brake()

    # Find Destinated Road Points
    roadPoints = LaneMap.findRoadPoints(slicedGlobalPathPoints, potential)

    velocity = Cruise.velocity(slicedGlobalPathPoints)
    steering = Cruise.steering(roadPoints)

    # DRIVE
    Vehicle.accel(velocity)
    Vehicle.steerRadian(steering)

    if DEBUG:


        Lidar.publishPointCloud()

        # Clone Map
        mapImg = colormap.copy()

        # Vehicle
        vehiclePoint = VehicleStatus.position
        drawPoint(mapImg, vehiclePoint.x, vehiclePoint.y, (255, 0, 0), 15)

        # Lidar
        # drawLidarOnMap(mapImg)

        # Draw Global Points
        for point in GlobalPath.getGlobalPath():
            drawPoint(mapImg, point.x, point.y, (255, 255, 0), 5)

        lanePoints, distances = LaneMap.findNearestLanePoints(
            simPoints=slicedGlobalPathPoints
        )

        for point in lanePoints[LaneType.CENTER.value]:
            drawPoint(mapImg, point[0], point[1], (0, 0, 255), 5)

        for point in lanePoints[LaneType.DOT.value]:
            drawPoint(mapImg, point[0], point[1], (0, 0, 255), 5)

        for point in lanePoints[LaneType.EDGE.value]:
            drawPoint(mapImg, point[0], point[1], (0, 0, 255), 5)

        # for point1, point2 in zip(
        #     lanePoints[LaneType.EDGE.value], lanePoints[LaneType.DOT.value]
        # ):
        #     midX = point1[0] + point2[0]
        #     midY = point1[0] + point2[0]
        #     drawPoint(mapImg, midX, midY, (0, 0, 255), 5)

        # Draw DOT lane Points
        for point in roadPoints:
            drawPoint(mapImg, point[0], point[1], (255, 0, 255), 5)

        vx, vy = LaneMap.convertPointSim2Img(
            VehicleStatus.position.x, VehicleStatus.position.y
        )

        # plt.imshow(LaneMap.MAP)
        # plt.show()

        # cv2 window
        cv2.imshow("img", mapImg)
        cv2.waitKey(1)
