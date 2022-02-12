from math import radians
from typing import List

import cv2
import rospy
from geometry_msgs.msg import Point32

import GlobalPath
import LaneMap
import Vehicle
from Subscribers import Lidar, VehicleStatus
from utils import getFilePath

rospy.init_node("draw_on_map", anonymous=True)

# Load Global Path
globalPathFile = getFilePath("path/global_path.txt")
GlobalPath.load(globalPathFile)

# Load Colored Map for Debugging
colorMapFile = getFilePath("mapimg/colorLabeledMap.png")
colormap = cv2.imread(colorMapFile, cv2.IMREAD_ANYCOLOR)

while not rospy.is_shutdown():
    # Clone Map
    mapImg = colormap.copy()

    vehiclePoint = VehicleStatus.position
    x, y = LaneMap.convertPointSim2Img(vehiclePoint.x, vehiclePoint.y)
    if x is None or y is None:
        continue

    # Draw Vehicle
    cv2.rectangle(
        mapImg,
        (x - 8, y - 8),
        (x + 8, y + 8),
        255,
        -1,
    )

    # Retrieve Lidar Points
    Lidar.publishPointCloud()
    lidarPoints = Lidar.convert2Points(
        radians(VehicleStatus.heading)
    )  # type: List[Point32]
    for lidarPoint in lidarPoints:
        absLidarPoint = vehiclePoint.x + lidarPoint.x, vehiclePoint.y + lidarPoint.y
        absLidarPointOnMapX, absLidarPointOnMapY = LaneMap.convertPointSim2Img(
            absLidarPoint[0], absLidarPoint[1]
        )
        if absLidarPointOnMapX is not None and absLidarPointOnMapY is not None:
            cv2.line(
                mapImg,
                (absLidarPointOnMapX, absLidarPointOnMapY),
                (absLidarPointOnMapX, absLidarPointOnMapY),
                255,
                3,
            )

    cv2.imshow("img", mapImg)
    cv2.waitKey(1)
