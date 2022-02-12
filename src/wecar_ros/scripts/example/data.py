#!/usr/bin/env python3

from math import sqrt
import rospy
from morai_msgs.msg import EgoVehicleStatus
import os


REFRESH_RATE = 30


class Data:
    def __init__(self):
        rospy.init_node("data", anonymous=True)

        rospy.Subscriber("/Ego_topic", EgoVehicleStatus, self.egoTopicCallback)
        rospy.spin()

    def egoTopicCallback(self, data):
        rate = rospy.Rate(REFRESH_RATE)
        rate.sleep()

        os.system("clear")
        # print(data)

        acceleration = data.acceleration
        print(
            "Acceleration: {0:.3f}".format(
                sqrt(acceleration.x ** 2 + acceleration.y ** 2)
            )
        )
        print("x: {0:.3f}".format(acceleration.x))
        print("y: {0:.3f}".format(acceleration.y))
        print("z: {0:.3f}".format(acceleration.z))

        print("Position")
        position = data.position
        print("x: {0:.3f}".format(position.x))
        print("y: {0:.3f}".format(position.y))
        print("z: {0:.3f}".format(position.z))

        print(
            "Velocity: {0:.3f}".format(
                sqrt(data.velocity.x ** 2 + data.velocity.y ** 2)
            )
        )
        velocity = data.velocity
        print("x: {0:.3f}".format(velocity.x))
        print("y: {0:.3f}".format(velocity.y))
        print("z: {0:.3f}".format(velocity.z))

        print(f"Heading: {data.heading}")
        print(f"Accel: {data.accel}")
        print(f"Brake: {data.brake}")
        print(f"Wheel_Angle: {data.wheel_angle}")


if __name__ == "__main__":
    try:
        data = Data()
    except rospy.ROSInterruptException:
        pass
