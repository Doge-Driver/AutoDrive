#!/usr/bin/env python3

import cv2
import numpy as np
import rospy
from sensor_msgs.msg import CompressedImage, Image
from matplotlib import pyplot as plt


class IMGParser:
    def __init__(self):
        rospy.init_node("camera", anonymous=True)
        self.image_sub = rospy.Subscriber(
            "/image_jpeg/compressed", CompressedImage, self.callback
        )

        rospy.spin()

    def callback(self, data):
        np_arr = np.fromstring(data.data, np.uint8)
        img_bgr = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
        # cv2.imshow("Image window", img_bgr)
        # cv2.waitKey(1)
        plt.imshow(img_bgr)
        plt.show()


if __name__ == "__main__":
    try:
        image_parser = IMGParser()
    except rospy.ROSInterruptException:
        pass
