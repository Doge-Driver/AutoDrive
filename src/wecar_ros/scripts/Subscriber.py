#! /usr/bin/python3
from math import cos, pi, radians, sin
from time import time
from typing import Generic, List, TypeVar

import cv2
import numpy as np
import rospy
from geometry_msgs.msg import Point32
from morai_msgs.msg import (
    EgoVehicleStatus,
    GetTrafficLightStatus,
    ObjectStatus,
    ObjectStatusList,
)
from sensor_msgs.msg import CompressedImage, Imu, LaserScan, PointCloud

from lib.utils import SingletonInstance

T = TypeVar("T")


class Subscriber(Generic[T]):
    TIMEOUT = 1

    def __init__(self, topic, msgType):
        self.__topic = topic
        self.__type = msgType
        self.__data = msgType()
        self.retrieve()

    def retrieve(self):  # type: () -> Subscriber[T]
        """Retrives Data from ROS

        Returns retrived data
        """
        self.isDataRetrieved = False
        self.__subscriber = rospy.Subscriber(self.__topic, self.__type, self.__set)
        return self

    def __set(self, data):
        self.isDataRetrieved = True
        self.__data = data  # type: T
        self.__subscriber.unregister()

    def get(self):  # type: () -> T
        startTime = time()
        while not self.isDataRetrieved:
            if time() - startTime > self.TIMEOUT:
                break
            continue

        return self.__data


class Camera(Subscriber[CompressedImage], SingletonInstance):
    def __init__(self):
        super().__init__("/image_jpeg/compressed", CompressedImage)

    def retrieveImage(self):
        self.retrieve()
        return self.getImage()

    def getImage(self):  # type: () -> np.ndarray
        np_arr = np.frombuffer(self.get().data, dtype=np.uint8)
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


class Lidar(Subscriber[LaserScan], SingletonInstance):
    ANGLE_YAW = pi

    def __init__(self):
        super().__init__("/lidar2D", LaserScan)
        self.pcd_pub = rospy.Publisher("laser2pcd", PointCloud, queue_size=1)

    def calcPoints(self):  # type: () -> PointCloud
        pcd = PointCloud()
        pcd.header.frame_id = self.get().header.frame_id

        angleIncrement = self.get().angle_increment
        angle = 0.0

        ranges = self.get().ranges

        for range in ranges:
            if range < 10.0:
                pcd.points.append(
                    Point32(
                        x=range * cos(angle),  # + vehicleStatus.position.x,
                        y=range * sin(angle),  # + vehicleStatus.position.y,
                    )
                )
            angle += angleIncrement
        return pcd

    def publish(self):
        pcd = self.calcPoints()
        self.pcd_pub.publish(pcd)


class IMU(Subscriber[Imu], SingletonInstance):
    def __init__(self):
        super().__init__("/imu", Imu)


class ObjectInfo(Subscriber[ObjectStatusList], SingletonInstance):
    def __init__(self):
        super().__init__("/Object_topic", ObjectStatusList)

    def getNpcs(self):  # type: () -> List[ObjectStatus]
        return self.get().npc_list

    def getPedestrians(self):  # type: () -> List[ObjectStatus]
        return self.get().pedestrian_list

    def getObstacles(self):  # type: () -> List[ObjectStatus]
        return self.get().obstacle_list

    def getList(self):  # type: () -> List[ObjectStatus]
        return self.getNpcs() + self.getPedestrians() + self.getObstacles()


class TrafficLight(Subscriber[GetTrafficLightStatus], SingletonInstance):
    def __init__(self):
        super().__init__("/GetTrafficLightStatus", GetTrafficLightStatus)

    def isRed(self):
        return (self.get().trafficLightStatus == 1) or (
            self.get().trafficLightStatus == 33
        )

    def isGreen(self):
        return self.get().trafficLightStatus == 16

    def isYellow(self):
        return (self.get().trafficLightStatus == 4) or (
            self.get().trafficLightStatus == 5
        )

    def isLeftGreen(self):
        return self.get().trafficLightStatus == 33


class VehicleStatus(Subscriber[EgoVehicleStatus], SingletonInstance):
    def __init__(self):
        super().__init__("/Ego_topic", EgoVehicleStatus)
