#! /usr/bin/env python3

import rospy
from std_msgs.msg import Float64

from lib.utils import SingletonInstance
from Subscriber import *


class Vehicle(SingletonInstance):
    VEHICLE_LENGTH = 0.7
    VEHICLE_WIDTH = 0.2

    RATE_HZ = 30
    MAX_SPEED = 10000

    def __init__(self) -> None:
        self.motorPublisher = rospy.Publisher(
            "commands/motor/speed", Float64, queue_size=1
        )
        self.steerPublisher = rospy.Publisher(
            "commands/servo/position", Float64, queue_size=1
        )

    def brake(self):
        self.accel(0)

    def accel(self, speed=MAX_SPEED):
        """Set Vehicle Speed

        speed: Float64
        returns: None
        """
        self.motorPublisher.publish(speed)

    def steer(self, angle=0.5):
        """Set Steering Angle

        angle: Float64 between 0.0 ~ 1.0
        returns: None
        """
        self.steerPublisher.publish(angle)


from time import time

if __name__ == "__main__":
    rospy.init_node("test", anonymous=True)
    doge = Vehicle()
    startTime = time()
    while True:
        doge.steer(1)
        if time() - startTime >= 3:
            break
