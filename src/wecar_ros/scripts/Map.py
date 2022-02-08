import cv2
import numpy as np
from matplotlib import pyplot as plt

from Subscriber import *


class Map(SingletonInstance):
    ACTUAL_HEIGHT = 12
    ACTUAL_WIDTH = 40

    def __init__(self):
        mapImg = cv2.imread("map.png", cv2.IMREAD_GRAYSCALE)
        # 해상도에 맞게 알아서 사이즈 바꿔서 쓸것 (대신 10:3 비율 dsize)
        # map = cv2.resize(map, dsize=(1920, 576), interpolation=cv2.INTER_LINEAR)

        self.__mapImg = mapImg
        self.__mapImgHeight, self.__mapImgWidth = mapImg.shape

        self.__scaleFactor = {
            "x": self.__mapImgWidth / self.ACTUAL_WIDTH,
            "y": self.__mapImgHeight / self.ACTUAL_HEIGHT,
        }

    def convertPoint(self, x, y):
        if x > self.ACTUAL_WIDTH / 2 or x < -self.ACTUAL_WIDTH / 2:
            return None, None
        if y > self.ACTUAL_HEIGHT / 2 or y < -self.ACTUAL_HEIGHT / 2:
            return None, None
        scaleX, scaleY = x * self.__scaleFactor["x"], y * self.__scaleFactor["y"]

        return int(self.__mapImgWidth / 2 + scaleX), int(
            self.__mapImgHeight / 2 - scaleY
        )

    def getMap(self):
        return self.__mapImg.copy()


rospy.init_node("draw_on_map", anonymous=True)
map = Map()
while not rospy.is_shutdown():
    # Draw
    VehicleStatus().retrieve()
    vehiclePosition = VehicleStatus().get().position
    x, y = map.convertPoint(vehiclePosition.x, vehiclePosition.y)
    if x is None or y is None:
        continue

    mapImg = map.getMap()

    cv2.rectangle(
        mapImg,
        (x - 3, y - 3),
        (x + 3, y + 3),
        255,
        -1,
    )

    cv2.imshow("img", mapImg)
    cv2.waitKey(1)
