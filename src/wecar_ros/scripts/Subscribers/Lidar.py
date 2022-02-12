from math import cos, pi, radians, sin
from typing import List

import rospy
from geometry_msgs.msg import Point32
from sensor_msgs.msg import LaserScan, PointCloud
from std_msgs.msg import Header

__isRetrieved = False
__pcd_pub = rospy.Publisher("laser2pcd", PointCloud, queue_size=1)

ANGLE_YAW = pi

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


def convert2Points(angleOffset=0):
    angle = ANGLE_YAW + angleOffset

    points = []  # type: List[Point32]

    for range in ranges:
        if range < 10.0:
            points.append(
                Point32(
                    x=range * cos(angle),  # + vehicleStatus.position.x,
                    y=range * sin(angle),  # + vehicleStatus.position.y,
                )
            )
        angle += angleIncrement

    return points


def publishPointCloud():
    global __pcd_pub
    pcd = PointCloud()
    pcd.header.frame_id = header.frame_id
    pcd.points = convert2Points()
    __pcd_pub.publish(pcd)


def __setLidar(res):  # type: (LaserScan) -> None
    global __isRetrieved, header, angleMin, angleMax, angleIncrement, timeIncrement, scanTime, rangeMin, rangeMax, ranges, intensities
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


rospy.Subscriber("/lidar2D", LaserScan, __setLidar)
