import rospy
from morai_msgs.msg import GetTrafficLightStatus
from std_msgs.msg import Header

__isRetrieved = False

header = Header()
trafficLightIndex = ""
trafficLightType = 0
trafficLightStatus = 0


def __setTrafficLight(res):  # type: (GetTrafficLightStatus) -> None
    global __isRetrieved, header, trafficLightIndex, trafficLightType, trafficLightStatus
    __isRetrieved = True
    header = res.header
    trafficLightIndex = res.trafficLightIndex
    trafficLightType = res.trafficLightType
    trafficLightStatus = res.trafficLightStatus


def isRed():
    global trafficLightStatus
    return (trafficLightStatus == 1) or (trafficLightStatus == 33)


def isGreen():
    global trafficLightStatus
    return trafficLightStatus == 16


def isYellow():
    global trafficLightStatus
    return (trafficLightStatus == 4) or (trafficLightStatus == 5)


def isLeftGreen():
    global trafficLightStatus
    return trafficLightStatus == 33


rospy.Subscriber("/GetTrafficLightStatus", GetTrafficLightStatus, __setTrafficLight)
