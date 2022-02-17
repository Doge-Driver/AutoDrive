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
# GlobalPath.load("path/global_path4.txt")
GlobalPath.load("path/object_test.txt")

doneMission1 = False
mission1Time = 0.0

potentialFile = getFilePath("mapimg/Potential_v3.PNG")
potential = cv2.imread(potentialFile, cv2.IMREAD_UNCHANGED)

while not rospy.is_shutdown():
    # GlobalPath
    if GlobalPath.isLastPathIndex():
        Vehicle.stop()
        rospy.signal_shutdown("GOAL")
        break
    GlobalPath.updatePathIndex()

    # Update Lane Info Nearby Vehicle
    (
        (leftLane, leftLanePoint, leftLaneDistance),
        (frontLane, frontLanePoint, frontLaneDistance),
        (rightLane, rightLanePoint, rightLaneDistance),
    ) = Vehicle.updateNearbyLanes()

    # MISSION1: Wait for 5 sec at second stop line
    if (
        not doneMission1
        and Vehicle.distanceWith(LaneMap.convertPointImg2Sim(3024, 580)) < 1.0
    ):
        if mission1Time == 0.0:
            mission1Time = time.time()
        elif time.time() - mission1Time < 5:
            velocity = 0
            steering = 0
        else:
            doneMission1 = True

    # MISSION2: Wait for Rotary vehicles

    # MISSION3: Traffic Light
    elif LaneMap.isIntersection() and not TrafficLight.isLeftGreen():
        velocity = 0
        steering = 0

    # MISSION4: Avoid Static Obstacles
    elif Obstacle.isForward():
        print("obstacle detected!!")
        evasionPoint = Obstacle.getEvasionPoint()
        print(f"evasion point {evasionPoint}")
        velocity = Cruise.velocity([evasionPoint])
        steering = Cruise.steering([evasionPoint])

    # MISSION5: Emergency Brake when Dynamic Obstacles
    elif Obstacle.isDynamicObstacleDetected():
        velocity = 0
        steering = 0

    else:
        # Cruise
        slicedGlobalPathPoints = GlobalPath.getSlicedGlobalPath(10)
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
        drawLidarOnMap(mapImg)

        # Draw Global Points
        for point in GlobalPath.getGlobalPath()[GlobalPath.getCurrentPathIndex() :]:
            drawPoint(mapImg, point.x, point.y, (255, 255, 0), 5)

        # Draw DOT lane Points
        for point in roadPoints:
            drawPoint(mapImg, point[0], point[1], (255, 0, 255), 5)

        # cv2 window
        cv2.imshow("img", mapImg)
        cv2.waitKey(1)
