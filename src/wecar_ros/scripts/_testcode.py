import rospy

from subscribers import lidar

rospy.init_node("hello", anonymous=True)

while not rospy.is_shutdown():
    print(lidar.ranges)
