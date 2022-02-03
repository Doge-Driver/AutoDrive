import sys

import cv2
import matplotlib.pyplot as plt
import numpy as np
import rospy

from GlobalPath import GlobalPath
from Subscriber import *
from Vehicle import Vehicle


class DogeDriver:
    def __init__(
        self,
        vehicle: Vehicle,
        globalPath: GlobalPath,
        objectInfo: ObjectInfo,
        lane,
        trafficLight: TrafficLight,
        camera: Camera,
    ):
        self.vehicle = vehicle
        self.path = globalPath
        self.objectInfo = objectInfo
        self.lane = lane
        self.trafficLight = trafficLight
        self.camera = camera


if __name__ == "__main__":
    rospy.init_node("doge_driver", anonymous=True)

    """Set Vehicle"""
    tesla = Vehicle()

    """Set Inputs"""
    camera = Camera()
    imu = IMU()
    objectInfo = ObjectInfo()
    trafficLight = TrafficLight()
    vehicleStatus = VehicleStatus()

    """Set GlobalPath"""
    globalPathFile = "test_path1"
    # globalPathFile = rospy.myargv(argv=sys.argv)[1]
    globalPath = GlobalPath(vehicleStatus, globalPathFile)

    """Set LaneDetection"""
    # TODO: Initialize LANE DETECTION

    # musk = DogeDriver(tesla, globalPath, objectInfo, lane, trafficLight, camera)

    while True:
        image = camera.retrieveImage()
        cv2.imshow("test", image)
        cv2.waitKey(1)
        rospy.Rate(30).sleep()
