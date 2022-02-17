from math import cos, pi, radians, sin
from typing import List

import LaneMap
import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Header

from . import VehicleStatus

__isRetrieved = False
__pcd_pub = rospy.Publisher("laser2pcd", PointCloud, queue_size=1)

ANGLE_YAW = pi
RANGE_MAX = 10.0

header = Header()
angleMin = 0.0
angleMax = 0.0
angleIncrement = 0.0

timeIncrement = 0.0
scanTime = 0.0
rangeMin = 0.0
rangeMax = 0.0

ranges = []  # type: List[float]
intensities = []

rotatedRanges = []
filteredRanges = []


def convert2Points(angleOffset=0):
    angle = ANGLE_YAW + angleOffset

    points = []  # type: List[Point32]

    for range in ranges:
        if range < RANGE_MAX:
            points.append(
                Point32(
                    x=range * cos(angle),  # + vehicleStatus.position.x,
                    y=range * sin(angle),  # + vehicleStatus.position.y,
                )
            )
        angle += angleIncrement

    return points


def abscartpoint():
    cartesianRelativeLidarPoints = convert2Points(
        angleOffset=radians(VehicleStatus.heading)
    )  # type: List[Point32]
    cartesianAbsoluteLidarPoints = []

    for point in cartesianRelativeLidarPoints:
        cartesianAbsoluteLidarPoints.append(
            (VehicleStatus.position.x + point.x, VehicleStatus.position.y + point.y)
        )


def filterRanges(ranges=ranges):
    global filteredRanges
    angle = ANGLE_YAW + radians(VehicleStatus.heading)
    filteredRanges = []

    for range in ranges:
        if range > RANGE_MAX:
            filteredRanges.append(RANGE_MAX)
            continue
        x = range * cos(angle) + VehicleStatus.position.x
        y = range * sin(angle) + VehicleStatus.position.y
        angle += angleIncrement
        if not LaneMap.inSimRange(x, y):
            filteredRanges.append(RANGE_MAX)
        else:
            filteredRanges.append(range)

    return filteredRanges


def publishPointCloud():
    global __pcd_pub
    pcd = PointCloud()
    pcd.header.frame_id = header.frame_id
    pcd.points = convert2Points()
    __pcd_pub.publish(pcd)


def __setLidar(res):  # type: (LaserScan) -> None
    global __isRetrieved, header, angleMin, angleMax, angleIncrement, timeIncrement, scanTime, rangeMin, rangeMax, ranges, intensities, rotatedRanges
    __isRetrieved = True
    header = res.header
    angleMin = res.angle_min
    angleMax = res.angle_max
    angleIncrement = res.angle_increment
    timeIncrement = res.time_increment
    scanTime = res.scan_time
    rangeMin = res.range_min
    rangeMax = res.range_max
    ranges = res.ranges
    intensities = res.intensities

    rotatedRanges = ranges[180:360] + ranges[0:180]
    filterRanges()


rospy.Subscriber("/lidar2D", LaserScan, __setLidar)
