#! /usr/bin/env python3

from math import asin, radians

import rospy
from std_msgs.msg import Float64

from lib.utils import SingletonInstance


class Vehicle(SingletonInstance):
    VEHICLE_LENGTH = 0.43
    VEHICLE_WIDTH = 0.2
    VEHICLE_WHEEL_BASE = 0.36162852

    STEER_LIMIT_DEG = 19.48
    STEER_MAX_DEG, STEER_MIN_DEG = STEER_LIMIT_DEG, -STEER_LIMIT_DEG
    STEER_LIMIT_RAD, STEER_MAX_RAD, STEER_MIN_RAD = (
        radians(STEER_LIMIT_DEG),
        radians(STEER_MAX_DEG),
        radians(STEER_MIN_DEG),
    )

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
        if angle > 1:
            angle = 1
        elif angle < 0:
            angle = 0
        self.steerPublisher.publish(angle)

    def steerRadian(self, radianAngle=0):
        """
        +: Steer Right
        -: Steer Left
        """
        angle = 0.5 + 0.5 * radianAngle / self.STEER_LIMIT_RAD
        self.steerPublisher.publish(angle)

    def steerRadius(self, radius):
        # radius
        if -self.VEHICLE_WHEEL_BASE < radius < self.VEHICLE_WHEEL_BASE:
            radianAngle = 0
        else:
            radianAngle = asin(self.VEHICLE_WHEEL_BASE / radius)
        self.steerRadian(radianAngle)


if __name__ == "__main__":
    rospy.init_node("test", anonymous=True)
    doge = Vehicle()
    while not rospy.is_shutdown():
        # doge.steerRadius(1.084)
        doge.steerRadius(-1.8961)
        doge.accel()
