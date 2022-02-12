from math import sqrt
from rospkg import RosPack


def getFilePath(fileName):
    packageLocation = RosPack().get_path("wecar_ros")
    fileName = f"{packageLocation}/{fileName}"
    return fileName


def distance(position1, position2):
    return sqrt((position1.x - position2.x) ** 2 + (position1.y - position2.y) ** 2)
