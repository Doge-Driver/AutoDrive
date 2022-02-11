#! /usr/bin/python3

from enum import Enum
from math import sqrt

import cv2
import numpy as np
from rospkg import RosPack

from Subscriber import *


def getImgFilePath(imgFileName):
    packageLocation = RosPack().get_path("wecar_ros")
    fileName = f"{packageLocation}/mapimg/{imgFileName}"
    return fileName


class LaneType(Enum):
    EDGE = 1
    DOT = 2
    CENTER = 3
    STOP = 4


class LaneMap(SingletonInstance):
    SIM_WIDTH, SIM_HEIGHT = 40, 12
    SIM_MAX_X, SIM_MAX_Y = SIM_WIDTH / 2, SIM_HEIGHT / 2
    SIM_MIN_X, SIM_MIN_Y = -SIM_WIDTH / 2, -SIM_HEIGHT / 2

    def __init__(self):
        file = getImgFilePath("labeledMap.png")
        map = cv2.imread(file, cv2.IMREAD_GRAYSCALE)  # type: np.ndarray
        self.__map = map

        self.IMG_HEIGHT, self.IMG_WIDTH = map.shape[:2]
        self.IMG_MAX_X, self.IMG_MAX_Y = self.IMG_WIDTH, self.IMG_HEIGHT
        self.IMG_MIN_X, self.IMG_MIN_Y = 0, 0
        self.IMG_CENTER_X, self.IMG_CENTER_Y = int(self.IMG_WIDTH / 2), int(
            self.IMG_HEIGHT / 2
        )

        self.SCALE_FACTOR = (
            self.IMG_WIDTH / self.SIM_WIDTH
        )  # IMG_WIDTH / SIM_WIDTH == IMG_HEIGHT / SIM_HEIGHT

    def convertSizeSim2Img(self, simSize):
        return int(simSize * self.SCALE_FACTOR)

    def convertSizeImg2Sim(self, imgSize):
        return imgSize / self.SCALE_FACTOR

    def convertPointSim2Img(
        self, x, y
    ):  # type: (int, int) -> tuple[int, int] | tuple[None, None]
        if x > self.SIM_MAX_X or x < self.SIM_MIN_X:
            return None, None
        if y > self.SIM_MAX_Y or y < self.SIM_MIN_Y:
            return None, None
        scaledX, scaledY = x * self.SCALE_FACTOR, y * self.SCALE_FACTOR
        return int(self.IMG_CENTER_X + scaledX), int(self.IMG_CENTER_Y - scaledY)

    def convertPointImg2Sim(
        self, x, y
    ):  # type: (int, int) -> tuple[int, int] | tuple[None, None]
        if x > self.IMG_MAX_X or x < self.IMG_MIN_X:
            return None, None
        if x > self.IMG_MAX_Y or x < self.IMG_MIN_Y:
            return None, None
        scaledX, scaledY = x / self.SCALE_FACTOR, y / self.SCALE_FACTOR
        return self.SIM_MIN_X + scaledX, self.SIM_MAX_Y - scaledY

    def getMap(self):
        return self.__map

    def findNearestLanePoint(self, simX, simY, ksize=0.5):  # SimScaled
        imgX, imgY = self.convertPointSim2Img(simX, simY)
        kSizeX, kSizeY = self.convertSizeSim2Img(ksize), self.convertSizeSim2Img(ksize)

        minDistance = {
            LaneType.EDGE.value: 10000,
            LaneType.DOT.value: 10000,
            LaneType.CENTER.value: 10000,
            LaneType.STOP.value: 10000,
        }  # type: dict[int, tuple[int, int]]

        nearestPoint = {
            LaneType.EDGE.value: (-1, -1),
            LaneType.DOT.value: (-1, -1),
            LaneType.CENTER.value: (-1, -1),
            LaneType.STOP.value: (-1, -1),
        }  # type: dict[int, tuple[int, int]]

        for index, value in np.ndenumerate(
            self.__map[
                imgX - kSizeX : imgX + kSizeX,
                imgY - kSizeY : imgY + kSizeY,
            ]
        ):
            # 안해도 안전할거라 믿어
            # if not value in LaneType.name:
            #     continue
            iy, ix = index
            distance = sqrt((imgX - ix) ** 2 + (imgY - iy) ** 2)

            if minDistance[value] > distance:
                minDistance[value] = self.convertSizeImg2Sim(distance)
                nearestPoint[value] = self.convertPointImg2Sim(iy, ix)

        return nearestPoint, minDistance

    def findNearestLanePoints(self, simPoints):  # SimScaled
        lanePoints = (
            []
        )  # type: List[tuple[dict[int, tuple[int, int]], dict[int, tuple[int, int]]]]
        for point in simPoints:
            x, y = point
            lanePoints.append(self.findNearestLanePoint(x, y))

        return lanePoints

    def findRoadPoint(self, simX, simY, distanceFromLane=0.15):  # SimScaled
        imgX, imgY = self.convertPointSim2Img(simX, simY)
        convertedDistanceFromLane = self.convertSizeSim2Img(distanceFromLane)
        nearestPoint, _ = self.findNearestLanePoint(simX, simY)

        roadPoint = {
            LaneType.EDGE.value: (-1, -1),
            LaneType.DOT.value: (-1, -1),
            LaneType.CENTER.value: (-1, -1),
            LaneType.STOP.value: (-1, -1),
        }

        for key, value in nearestPoint.items():
            simX, simY = value
            if simX == -1 or simY == -1:
                continue
            vectorSize = sqrt((imgX - simX) ** 2 + (imgY - simY) ** 2)

            vectorX = (imgX - simX) * convertedDistanceFromLane / vectorSize
            vectorY = (imgY - simY) * convertedDistanceFromLane / vectorSize
            roadPoint[key] = self.convertSizeImg2Sim(
                imgX + vectorX
            ), self.convertSizeImg2Sim(imgY + vectorY)

        return roadPoint

    def findRoadPoints(self, simPoints):  # SimScaled
        roadPoints = []  # type: List[dict[int, tuple[int, int]]]
        for point in simPoints:
            x, y = point
            roadPoints.append(self.findRoadPoint(x, y))

        return roadPoints


rospy.init_node("draw_on_map", anonymous=True)
map = LaneMap()
file = getImgFilePath("colorLabeledMap.png")
colormap = cv2.imread(file, cv2.IMREAD_ANYCOLOR)

while not rospy.is_shutdown():
    # Clone Map
    mapImg = colormap.copy()

    # Retrieve & Convert Vehicle Point to Img Point
    VehicleStatus().retrieve()
    vehiclePoint = VehicleStatus().get().position
    x, y = map.convertPointSim2Img(vehiclePoint.x, vehiclePoint.y)
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
    # Lidar().retrieve()
    # lidarPoints = Lidar().calcPoints().points  # type: List[Point32]
    # for lidarPoint in lidarPoints:
    #     absLidarPoint = vehiclePoint.x + lidarPoint.x, vehiclePoint.y + lidarPoint.y
    #     absLidarPointOnMapX, absLidarPointOnMapY = map.convertPointSim2Img(
    #         absLidarPoint[0], absLidarPoint[1]
    #     )
    #     if absLidarPointOnMapX is not None and absLidarPointOnMapY is not None:
    #         cv2.line(
    #             mapImg,
    #             (absLidarPointOnMapX, absLidarPointOnMapY),
    #             (absLidarPointOnMapX, absLidarPointOnMapY),
    #             255,
    #             3,
    #         )

    cv2.imshow("img", mapImg)
    cv2.waitKey(1)
