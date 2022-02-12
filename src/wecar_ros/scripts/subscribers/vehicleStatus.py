import rospy
from geometry_msgs.msg import Vector3
from morai_msgs.msg import EgoVehicleStatus
from std_msgs.msg import Header

__isRetrieved = False

header = Header()
unique_id = 0
acceleration = Vector3()
position = Vector3()
velocity = Vector3()
heading = 0.0
accel = 0.0
brake = 0.0
wheel_angle = 0.0


def __setVehicleStatus(res):  # type: (EgoVehicleStatus) -> None
    global __isRetrieved, header, unique_id, acceleration, position, velocity, heading, accel, brake, wheel_angle

    __isRetrieved = True
    header = res.header
    unique_id = res.unique_id
    acceleration = res.acceleration
    position = res.position
    velocity = res.velocity
    heading = res.heading
    accel = res.accel
    brake = res.brake
    wheel_angle = res.wheel_angle


rospy.Subscriber("/Ego_topic", EgoVehicleStatus, __setVehicleStatus)
