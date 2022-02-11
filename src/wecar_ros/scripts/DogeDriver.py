import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy

from GlobalPath import GlobalPath
from LaneMap import LaneMap
from Subscriber import *
from Vehicle import Vehicle


class DogeDriver:
    def __init__(
        self,
        vehicle: Vehicle,
        globalPath: GlobalPath,
        laneMap,
    ):
        self.vehicle = vehicle
        self.path = globalPath
        self.laneMap = laneMap

    def followLane(self):
        pass

    def drive(self):
        pass


if __name__ == "__main__":
    rospy.init_node("doge_driver", anonymous=True)

    # Initialize Vehicle
    tesla = Vehicle()

    # Initialize Inputs
    vehicleStatus = VehicleStatus()
    camera = Camera()
    lidar = Lidar()
    trafficLight = TrafficLight()

    # Set GlobalPath
    globalPathFile = "test_path1"
    # globalPathFile = rospy.myargv(argv=sys.argv)[1]
    globalPath = GlobalPath(globalPathFile)

    # Set LaneMap
    laneMap = LaneMap()

    # Set ObstacleMap

    elonmusk = DogeDriver(tesla, globalPath, laneMap)

    while not rospy.is_shutdown():
        vehicleStatus.retrieve()
        lidar.retrieve()
        trafficLight.retrieve()

        elonmusk.drive()

        rospy.Rate(30).sleep()
