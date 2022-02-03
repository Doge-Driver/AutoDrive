#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rospy
from morai_msgs.msg import GPSMessage


class erp_gps:
    def __init__(self):
        rospy.init_node("gps", anonymous=True)

        # subscriber
        rospy.Subscriber("/gps", GPSMessage, self.gpsCB)
        rospy.spin()

    def gpsCB(self, data):
        # __slots__ = ['header','latitude','longitude','altitude','eastOffset','northOffset','status']
        # _slot_types = ['std_msgs/Header','float64','float64','float64','float64','float64','int16']

        print(f"latitude {data.latitude}")
        print(f"longitude {data.longitude}")
        print(f"eastOffset {data.eastOffset}")
        print(f"northOffset {data.northOffset}")


if __name__ == "__main__":
    try:
        gps = erp_gps()
    except rospy.ROSInterruptException:
        pass
