#! /usr/bin/python3

import time

import cv2
import rospy

import GlobalPath
import LaneMap
import Obstacle
import Vehicle
from Planner import Cruise
from Subscribers import Lidar, TrafficLight, VehicleStatus
from utils import getFilePath

rospy.init_node("doge_driver", anonymous=True)
rospy.on_shutdown(Vehicle.stop)

# Load Global Path
GlobalPath.load("path/global_path4.txt")

# Constants
SAFE_DISTANCE = 0.6

# Mission Flags
doneMission1 = False
mission1Time = 0.0

isEvading = False
evasionPoint = (0, 0)


# Load potential map
potentialFile = getFilePath("mapimg/Potential_v3.PNG")
potential = cv2.imread(potentialFile, cv2.IMREAD_UNCHANGED)

while not rospy.is_shutdown():
    # GlobalPath
    if GlobalPath.isLastPathIndex():
        Vehicle.stop()
        rospy.signal_shutdown("GOAL")
        break
    GlobalPath.updatePathIndex()

    # Cruise
    slicedGlobalPathPoints = GlobalPath.getSlicedGlobalPath(10)
    # Find Destinated Road Points
    roadPoints = LaneMap.findRoadPoints(slicedGlobalPathPoints, potential)

    velocity = Cruise.velocity(slicedGlobalPathPoints)
    steering = Cruise.steering(roadPoints)

    # Minimum Safe Distance
    minLidarDistance = min(Lidar.ranges[150:210])
    if minLidarDistance < 1.2:
        velocity = max((minLidarDistance - SAFE_DISTANCE) * 1000, 0)

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
    elif Vehicle.distanceWith(LaneMap.convertPointImg2Sim(2530, 462)) < 2.5:
        distanceWithCar = min(Lidar.ranges[180:240])

        # 차 오는지 판단
        if (
            Vehicle.distanceWith(LaneMap.convertPointImg2Sim(2530, 462)) > 2.0
            and distanceWithCar < 1.8
        ):
            velocity = 0

        # 차간거리 유지
        elif distanceWithCar < 1.0:
            velocity = max((distanceWithCar - SAFE_DISTANCE) * 1000, 0)

    # MISSION3: Traffic Light
    elif (
        Vehicle.distanceWith(LaneMap.convertPointImg2Sim(2063, 462)) < 2.5
        and not TrafficLight.isLeftGreen()
    ):
        velocity = 0
        steering = 0

    # MISSION 4 & 5: TRACK
    elif VehicleStatus.position.x < -0.3:
        MIN_VELOCITY = 1000

        # Mission 4: Static Obstacles
        if VehicleStatus.position.y > 0:
            if minLidarDistance < 2.0:
                velocity = max((minLidarDistance - SAFE_DISTANCE) * 1000, MIN_VELOCITY)
            if Obstacle.isForward():
                isEvading = True
                evasionPoint = Obstacle.getFirstEvasionPoint()

            if isEvading:
                velocity = MIN_VELOCITY
                steering = Cruise.steering([evasionPoint])
                if Vehicle.distanceWith(evasionPoint) < 0.5:
                    isEvading = False

        # MISSION5: Emergency Brake when Dynamic Obstacles
        # this will be done automatically if its not static obstacle
        # because of minimum safe distance

    # DRIVE
    Vehicle.accel(velocity)
    Vehicle.steerRadian(steering)
