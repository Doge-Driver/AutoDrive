from math import sqrt

from rospkg import RosPack


def getFilePath(fileName):
    packageLocation = RosPack().get_path("wecar_ros")
    fileName = f"{packageLocation}/{fileName}"
    return fileName


def calcDistance(position1, position2):
    p1x, p1y, p2x, p2y = 0, 0, 0, 0
    if type(position1) is tuple:
        p1x, p1y = position1
    else:
        p1x, p1y = position1.x, position1.y
    if type(position2) is tuple:
        p2x, p2y = position2
    else:
        p2x, p2y = position2.x, position2.y

    return sqrt((p1x - p2x) ** 2 + (p1y - p2y) ** 2)
