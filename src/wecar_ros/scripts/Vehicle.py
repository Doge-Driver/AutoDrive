#! /usr/bin/env python3

from math import asin, cos, degrees, radians, sin

import numpy as np
import rospy
from std_msgs.msg import Float64

import LaneMap
import Vehicle
from LaneMap import LaneType
from Subscribers import VehicleStatus
from utils import calcDistance

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


def stop():
    brake()
    steerRadian(0)


LEFT_LANE_ANGLE = radians(90)
FRONT_LANE_ANGLE = 0
RIGHT_LANE_ANGLE = radians(-90)


def getLane(angle, findRange=0.7):
    theta = -radians(VehicleStatus.heading) - angle
    imgRange = int(LaneMap.convertSizeSim2Img(findRange))

    imgVehicleX, imgVehicleY = LaneMap.convertPointSim2Img(
        VehicleStatus.position.x, VehicleStatus.position.y
    )

    for r in range(1, imgRange):
        x, y = r * cos(theta), r * sin(theta)  # Polar Coordinates to Cartesian
        x, y = int(x + imgVehicleX), int(y + imgVehicleY)
        if LaneMap.safeMapAccess(x, y) != 0:
            return LaneMap.safeMapAccess(x, y), LaneMap.convertSizeImg2Sim(r)
    return 0, None


frontLane, frontLaneDistance = 0, 0.0
leftLane, leftLaneDistance = 0, 0.0
rightLane, rightLaneDistance = 0, 0.0


def getFrontLane():
    global frontLane, frontLaneDistance
    frontLane, frontLaneDistance = getLane(FRONT_LANE_ANGLE, 0.9)
    return frontLane, frontLaneDistance


def getLeftLane():
    global leftLane, leftLaneDistance
    leftLane, leftLaneDistance = getLane(LEFT_LANE_ANGLE, 0.4)
    return leftLane, leftLaneDistance


def getRightLane():
    global rightLane, rightLaneDistance
    rightLane, rightLaneDistance = getLane(RIGHT_LANE_ANGLE, 0.4)
    return rightLane, rightLaneDistance


def updateNearbyLanes():
    return getLeftLane(), getFrontLane(), getRightLane()


def isOnStopLine():
    return frontLane == LaneType.STOP.value and frontLaneDistance < 0.6


def distanceWith(point):
    return calcDistance(VehicleStatus.position, point)


if __name__ == "__main__":
    rospy.init_node("vehicle_test", anonymous=True)
    while not rospy.is_shutdown():
        Vehicle.steerRadius(-1.8961)
        Vehicle.accel()
