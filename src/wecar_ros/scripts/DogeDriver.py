import rospy

import GlobalPath
import Vehicle
from Subscribers import Camera, Lidar, TrafficLight, VehicleStatus

rospy.init_node("doge_driver", anonymous=True)

while not rospy.is_shutdown():
    pass
