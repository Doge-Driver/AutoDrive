#! /usr/bin/python3
from time import time
from typing import Generic, TypeVar

import cv2
import numpy as np
import rospy
from morai_msgs.msg import EgoVehicleStatus, GetTrafficLightStatus, ObjectStatusList
from sensor_msgs.msg import CompressedImage, Imu, LaserScan

from lib.utils import SingletonInstance

T = TypeVar("T")


class Subscriber(Generic[T]):
    TIMEOUT = 1

    def __init__(self, topic, type) -> None:
        self.__topic = topic
        self.__type = type
        self.retrieve()

    def retrieve(self):  # type: () -> Subscriber[T]
        """Retrives Data from ROS

        Returns retrived data
        """
        self.__subscriber = rospy.Subscriber(self.__topic, self.__type, self.__set)
        return self

    def __set(self, data):
        self.isDataRetrieved = True
        self.__data = data  # type: T
        self.__subscriber.unregister()

    def get(self):  # type: () -> T
        startTime = time()
        while not self.isDataRetrieved:
            if time() - startTime >= self.TIMEOUT:
                return self.__type()

        return self.__data


class Camera(Subscriber[CompressedImage], SingletonInstance):
    def __init__(self):
        super().__init__("/image_jpeg/compressed", CompressedImage)

    def retrieveImage(self):
        self.retrieve()
        return self.getImage()

    def getImage(self):
        np_arr = np.frombuffer(self.get().data, dtype=np.uint8)
        if np_arr.size == 0:
            return None
        return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


class Lidar(Subscriber[LaserScan], SingletonInstance):
    def __init__(self):
        super().__init__("/lidar2D", LaserScan)


class IMU(Subscriber[Imu], SingletonInstance):
    def __init__(self):
        super().__init__("/imu", Imu)


class ObjectInfo(Subscriber[ObjectStatusList], SingletonInstance):
    def __init__(self):
        super().__init__("/Object_topic", ObjectStatusList)


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


class Subscribers:
    def __init__(self):
        self.camera = Camera()
        self.lidar = Lidar()
        self.imu = IMU()
        self.objectInfo = ObjectInfo()
        self.trafficLight = TrafficLight()
        self.vehicleInfo = VehicleStatus()


# if __name__ == "__main__":
#     rospy.init_node("test", anonymous=True)
#     # camera = Camera()
#     # print(camera.get())
#     imu = IMU()
#     while True:
#         # start_time = time()
#         # if imu.isDataRetrieved:
#         # print(imu.data)
#         print(imu.retrieve().get())
#         # print(1 / (time() - start_time))
