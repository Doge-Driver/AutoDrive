#! /usr/bin/env python3

from math import asin, radians

import rospy
from std_msgs.msg import Float64

import Vehicle

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

__motorPublisher = rospy.Publisher("commands/motor/speed", Float64, queue_size=1)
__steerPublisher = rospy.Publisher("commands/servo/position", Float64, queue_size=1)


def brake():
    accel(0)


def accel(speed=MAX_SPEED):
    """Set Vehicle Speed

    speed: Float64
    returns: None
    """
    __motorPublisher.publish(speed)


def steer(angle=0.5):
    """Set Steering Angle

    angle: Float64 between 0.0 ~ 1.0
    returns: None
    """
    if angle > 1:
        angle = 1
    elif angle < 0:
        angle = 0
    __steerPublisher.publish(angle)


def steerRadian(radianAngle=0):
    """
    +: Steer Right
    -: Steer Left
    """
    if radianAngle is None:
        radianAngle = 0
    angle = 0.5 + 0.5 * radianAngle / STEER_LIMIT_RAD

    __steerPublisher.publish(angle)


def steerRadius(radius):
    # radius
    if -VEHICLE_WHEEL_BASE < radius < VEHICLE_WHEEL_BASE:
        radianAngle = 0
    else:
        radianAngle = asin(VEHICLE_WHEEL_BASE / radius)
    steerRadian(radianAngle)


if __name__ == "__main__":
    rospy.init_node("vehicle_test", anonymous=True)
    while not rospy.is_shutdown():
        Vehicle.steerRadius(-1.8961)
        Vehicle.accel()
