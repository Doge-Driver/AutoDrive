import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import Header

__isRetrieved = False

header = Header()
format = ""
data = b""


def __setCamera(res):  # type: (CompressedImage) -> None
    global __isRetrieved, header, format, data
    __isRetrieved = True
    header = res.header
    format = res.format
    data = res.data


def getImage():
    if __isRetrieved == False:
        return None
    np_arr = np.frombuffer(data, dtype=np.uint8)
    return cv2.imdecode(np_arr, cv2.IMREAD_COLOR)


rospy.Subscriber("/image_jpeg/compressed", CompressedImage, __setCamera)
