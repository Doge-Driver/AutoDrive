from math import atan2, cos, radians, sin, sqrt

import numpy as np
from geometry_msgs.msg import Point
from Subscribers import VehicleStatus

lfd = 1
LFD_MIN = 0.3
LFD_MAX = 30
vehicle_length = 0.3


def steering_angle(path, velocity=2.4):
    global lfd
    vehicle_position = VehicleStatus.position
    vehicle_yaw = radians(VehicleStatus.heading)

    rotated_point = Point()
    is_look_forward_point = False

    for i in path:
        dx = i[0] - (vehicle_position.x + 0.065 * cos(vehicle_yaw))
        dy = i[1] - (vehicle_position.y + 0.065 * sin(vehicle_yaw))
        rr = sqrt(dx**2 + dy**2)
        if rr < 10:
            rotated_point.x = cos(vehicle_yaw) * dx + sin(vehicle_yaw) * dy
            rotated_point.y = sin(vehicle_yaw) * dx - cos(vehicle_yaw) * dy

            if rotated_point.x > 0:
                dis = sqrt(rotated_point.x**2 + rotated_point.y**2)
                if dis >= lfd:
                    lfd = velocity / 2.2
                    if lfd < LFD_MIN:
                        lfd = LFD_MIN
                    elif lfd > LFD_MAX:
                        lfd = LFD_MAX
                    is_look_forward_point = True

                    break

    theta = atan2(rotated_point.y, rotated_point.x)

    if is_look_forward_point:
        steering = atan2((2 * vehicle_length * sin(theta)), lfd)  # * 180 / pi
        return steering
    else:
        return None


def velocity_plan(road_point, velocity=3.0):  # road_point => shape =[n,2]
    slope_list = []
    for i in range(2):
        if len(road_point) <= i + 1:
            break
        if road_point[i].x == road_point[i + 1].y:
            slope = np.pi / 2
        else:
            slope = abs(
                atan2(
                    road_point[i].y - road_point[i + 1].y,
                    road_point[i].x - road_point[i + 1].x,
                )
            )
        slope_list.append(slope)
    if len(slope_list) == 0:
        return 2.0
    slope_min = min(slope_list)
    slope_max = max(slope_list)
    return velocity - 3.8 * (slope_max - slope_min)


def get_slicing_path(slicing_path):
    slicing_x = []
    slicing_y = []
    for point in slicing_path:
        slicing_x.append(point[0])
        slicing_y.append(point[1])
    dot_line_param = np.polyfit(slicing_x, slicing_y, 2)

    return dot_line_param


def poly(x, param):
    return param[0] * x**2 + param[1] * x + param[2]
