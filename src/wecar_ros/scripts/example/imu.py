#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import cv2
import numpy as np
from sensor_msgs.msg import Imu


class IMUParser:
    def __init__(self):
        rospy.init_node("imu", anonymous=True)
        self.image_sub = rospy.Subscriber("/imu", Imu, self.callback)
        rospy.spin()

    def callback(self, data):
        # __slots__ = ['header','orientation','orientation_covariance','angular_velocity','angular_velocity_covariance','linear_acceleration','linear_acceleration_covariance']
        # _slot_types = ['std_msgs/Header','geometry_msgs/Quaternion','float64[9]','geometry_msgs/Vector3','float64[9]','geometry_msgs/Vector3','float64[9]']

        rate = rospy.Rate(5)
        rate.sleep()
        print("orientation:")
        print(
            f"x : {data.orientation.x} y : {data.orientation.y} z : {data.orientation.z} w : {data.orientation.w}"
        )
        print("\nangular_velocity:")
        print(
            f"x : {data.angular_velocity.x} y : {data.angular_velocity.y} z : {data.angular_velocity.z}"
        )
        print("\nlinear_acceleration:")
        print(
            f"x : {data.linear_acceleration.x} y : {data.linear_acceleration.y} z : {data.linear_acceleration.z}"
        )
        print("\n===============================================================\n")


if __name__ == "__main__":
    try:
        imu_parser = IMUParser()
    except rospy.ROSInterruptException:
        pass
