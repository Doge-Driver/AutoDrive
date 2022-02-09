#! /usr/bin/python3

import math

import cv2
import numpy as np

import LineType
from Subscriber import *


class Map(SingletonInstance):
    SIM_WIDTH, SIM_HEIGHT = 40, 12
    SIM_MAX_X, SIM_MAX_Y = SIM_WIDTH / 2, SIM_HEIGHT / 2
    SIM_MIN_X, SIM_MIN_Y = -SIM_WIDTH / 2, -SIM_HEIGHT / 2

    def __init__(self):
        mapImg = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)
        # 해상도에 맞게 알아서 사이즈 바꿔서 쓸것 (대신 10:3 비율 dsize)
        # map = cv2.resize(map, dsize=(1920, 576), interpolation=cv2.INTER_LINEAR)

        self.__mapImg = mapImg
        self.IMG_HEIGHT, self.IMG_WIDTH = mapImg.shape
        self.IMG_MAX_X, self.IMG_MAX_Y = self.IMG_WIDTH, self.IMG_HEIGHT
        self.IMG_MIN_X, self.IMG_MIN_Y = 0, 0
        self.IMG_CENTER_X, self.IMG_CENTER_Y = int(self.IMG_WIDTH / 2), int(
            self.IMG_HEIGHT / 2
        )

        self.SCALE_FACTOR = {
            "x": self.IMG_WIDTH / self.SIM_WIDTH,
            "y": self.IMG_HEIGHT / self.SIM_HEIGHT,
        }

    def convertSizeXSim2Img(self, x):
        return x * self.SCALE_FACTOR["x"]

    def convertSizeYSim2Img(self, y):
        return y * self.SCALE_FACTOR["y"]

    def convertPointSim2Img(self, x, y):
        if x > self.SIM_MAX_X or x < self.SIM_MIN_X:
            return None, None
        if y > self.SIM_MAX_Y or y < self.SIM_MIN_Y:
            return None, None
        scaledX, scaledY = x * self.SCALE_FACTOR["x"], y * self.SCALE_FACTOR["y"]
        return int(self.IMG_CENTER_X + scaledX), int(self.IMG_CENTER_Y - scaledY)

    def convertPointImg2Sim(self, x, y):
        if x > self.IMG_MAX_X or x < self.IMG_MIN_X:
            return None, None
        if x > self.IMG_MAX_Y or x < self.IMG_MIN_Y:
            return None, None
        scaledX, scaledY = x / self.SCALE_FACTOR["x"], y / self.SCALE_FACTOR["y"]
        return self.SIM_MIN_X + scaledX, self.SIM_MIN_Y + scaledY

    def getMap(self):
        return self.__mapImg.copy()

    def findNearestPoint(self, x, y, lineType=LineType.EDGE):
        convertedX, convertedY = self.convertPointSim2Img(x, y)
        kSizeX, kSizeY = self.convertSizeXSim2Img(5), self.convertSizeYSim2Img(5)
        minDistance = 10000
        nearestPoint = (0, 0)
        for index, value in np.ndenumerate(
            self.__mapImg[
                convertedX - kSizeX : convertedX + kSizeX,
                convertedY - kSizeY : convertedY + kSizeY,
            ]
        ):
            ix, iy = index
            if value == lineType:
                distance = math.sqrt((convertedX - ix) ** 2 + (convertedY - iy) ** 2)
                if minDistance > distance:
                    minDistance = distance
                    nearestPoint = index

        return nearestPoint

    def findNearestPoints(self, points, lineType=LineType.EDGE):
        linePoints = []
        for point in points:
            x, y = point
            linePoints.append(self.findNearestPoint(x, y, lineType))

        return linePoints

    # def getPointDrawnMap(self, point, color=(255, 255, 255), thickness=3):
    #     mapImg = self.getMap()
    #     cv2.line(mapImg, point, point, color, thickness)
    #     return mapImg


rospy.init_node("draw_on_map", anonymous=True)
map = Map()
while not rospy.is_shutdown():
    # Retrieve & Convert Vehicle Point to Img Point
    VehicleStatus().retrieve()
    vehiclePoint = VehicleStatus().get().position
    x, y = map.convertPointSim2Img(vehiclePoint.x, vehiclePoint.y)
    if x is None or y is None:
        continue

    # Clone Map
    mapImg = map.getMap()

    # Draw
    cv2.rectangle(
        mapImg,
        (x - 3, y - 3),
        (x + 3, y + 3),
        255,
        -1,
    )

    cv2.imshow("img", mapImg)
    cv2.waitKey(1)
