from rospkg import RosPack


def getFilePath(fileName):
    packageLocation = RosPack().get_path("wecar_ros")
    fileName = f"{packageLocation}/{fileName}"
    return fileName
