#! /usr/bin/python3
# -*- coding: utf-8 -*-

from math import atan2, cos, pi, sin, sqrt

import cv2
import numpy as np
from geometry_msgs.msg import Point, PoseStamped


class Planner:
    def __init__(self, line_position, dot_line_position):
        self.__line_position = line_position
        self.__dot_line_position = dot_line_position

    def get_steering_angle(self, curvature, car_pos, car_heading):
        # L = 0.375
        steering = abs(atan2(curvature * 46.32, 1))
        heading = car_heading * np.pi / 180
        line_heading = atan2(
            self.__dot_line_position[len(self.__dot_line_position) - 1][1] - car_pos[1],
            self.__dot_line_position[len(self.__dot_line_position) - 1][0] - car_pos[0],
        )
        if heading > 0 and heading - line_heading < 0:
            steering = -steering
        elif heading < 0 and heading - line_heading < 0:
            steering = -steering

        steering = -steering / np.pi + 0.5

        return steering

    def curvature_rad(self):  # get line_position shape = (n,2)
        k = 0
        k1 = 0
        k2 = 0

        line_x = []
        line_y = []
        dot_line_x = []
        dot_line_y = []
        for i in range(len(self.__line_position)):
            line_x.append(self.__line_position[i][0])
            line_y.append(self.__line_position[i][1])

        for i in range(len(self.__dot_line_position)):
            dot_line_x.append(self.__dot_line_position[i][0])
            dot_line_y.append(self.__dot_line_position[i][1])

        dot_line_param = [0, 0, dot_line_x[0]]

        if abs(dot_line_x[0] - dot_line_x[1]) > 0:
            if len(self.__line_position) > 0:
                line_param = np.polyfit(line_x, line_y, 2)
                k1 = (
                    2
                    * line_param[0]
                    / pow(
                        sqrt(
                            (
                                pow(
                                    2 * line_param[0] * self.__line_position[0][0]
                                    + line_param[1],
                                    2,
                                )
                                + 1
                            )
                        ),
                        3,
                    )
                )
            if len(self.__dot_line_position) > 0:
                dot_line_param = np.polyfit(dot_line_x, dot_line_y, 2)
                k2 = (
                    2
                    * dot_line_param[0]
                    / pow(
                        sqrt(
                            (
                                pow(
                                    2
                                    * dot_line_param[0]
                                    * self.__dot_line_position[0][0]
                                    + dot_line_param[1],
                                    2,
                                )
                                + 1
                            )
                        ),
                        3,
                    )
                )
            if k1 > k2:
                k = k1
            else:
                k = k2
        # if len(line_position) == 0 and len(dot_line_position)==0:  ##차선없는 커브용 코드
        #   k=self.pp.steering_angle()
        return k, dot_line_param

    def get_fitline(self, lines):
        """line_list= []
        for ix,iy in lines:
            line_list.append([ix,iy])"""
        lines = np.array(lines)
        output = cv2.fitLine(lines, cv2.DIST_L2, 0, 0.01, 0.01)

        vx, vy, x, y = output[0], output[1], output[2], output[3]
        x1, y1 = (
            int(((lines[len(lines) - 1][1]) - y) / vy * vx + x),
            lines[len(lines) - 1][1],
        )  ##lines[len(lines)-1][1]
        x2, y2 = int(((lines[0][1]) - y) / vy * vx + x), lines[0][1]  ##lines[0][1]
        result = [x1, y1, x2, y2]
        return result

    def draw_fit_line(self, img, lines, color=[255, 255, 255], thickness=5):
        cv2.line(img, (lines[0], lines[1]), (lines[2], lines[3]), color, thickness)

    def get_bo_local_points(self, local_path, dot_line, dis):
        bo_list = []
        for i in range(len(dot_line)):  # points = local_path positions
            x = local_path[i][0]
            y = local_path[i][1]
            bo_list.append(self.get_bo_local_point(x, y, dot_line[i], dis))
        return bo_list

    def get_bo_local_point(self, x, y, dot_line_point, dis=15):
        lx, ly = dot_line_point
        if x == lx:
            bo_x = lx
            if ly > y:
                bo_y = ly - dis
            else:
                bo_y = ly + dis
        else:
            slope = (y - ly) / (x - lx)
            if lx > x:
                bo_x = lx - abs(dis * cos(slope))
            else:
                bo_x = lx + abs(dis * cos(slope))
            if ly > y:
                bo_y = ly - abs(dis * sin(slope))
            else:
                bo_y = ly + abs(dis * sin(slope))

        return [bo_x, bo_y]


class pidController:
    def __init__(self):
        self.p_gain = 0.91
        self.i_gain = 0.05
        self.d_gain = 0.07
        self.controlTime = 0.033
        self.prev_error = 0
        self.i = 0
        self.angle_list = [0.0, 0.0, 0.0]

    def pid(self, target_vel, current_vel, time_measure):
        error = target_vel - current_vel
        self.angle_list.append(error)
        del self.angle_list[0]
        i_error = self.simpson(self.angle_list, time_measure)
        p = self.p_gain * error
        # self.i+=self.i_gain*error*self.controlTime
        self.i += self.i_gain * i_error
        d = self.d_gain * (error - self.prev_error) / self.controlTime

        output = p + self.i + d
        self.prev_error = error
        return output

    def simpson(self, angles, h):
        return h / 3 * (angles[0] + 4 * angles[1] + angles[2])


class purePursuit:
    def __init__(self):
        self.forward_point = Point()
        self.current_postion = Point()
        self.is_look_forward_point = False
        self.lfd = 1
        self.min_lfd = 1
        self.max_lfd = 30
        self.vehicle_length = 0.3
        self.steering = 0

    def getPath(self, msg):
        self.path = msg  # nav_msgs/Path

    def getEgoStatus(self, msg):

        self.current_vel = msg.velocity.x  # kph
        self.vehicle_yaw = msg.heading / 180 * pi  # rad
        self.current_postion.x = msg.position.x
        self.current_postion.y = msg.position.y
        self.current_postion.z = msg.position.z

    def steering_angle(self):
        vehicle_position = self.current_postion
        rotated_point = Point()
        self.is_look_forward_point = False

        for i in self.path:
            dx = i[0] - vehicle_position.x
            dy = i[1] - vehicle_position.y
            rr = sqrt(pow(dx, 2) + pow(dy, 2))
            if rr < 10:
                rotated_point.x = (
                    cos(self.vehicle_yaw) * dx + sin(self.vehicle_yaw) * dy
                )
                rotated_point.y = (
                    sin(self.vehicle_yaw) * dx - cos(self.vehicle_yaw) * dy
                )

                if rotated_point.x > 0:
                    dis = sqrt(pow(rotated_point.x, 2) + pow(rotated_point.y, 2))
                    print("dis")
                    print("dis , lfd : ", dis, self.lfd)
                    if dis >= self.lfd:

                        self.lfd = self.current_vel / 3
                        if self.lfd < self.min_lfd:
                            self.lfd = self.min_lfd
                        elif self.lfd > self.max_lfd:
                            self.lfd = self.max_lfd
                        self.forward_point = i
                        self.is_look_forward_point = True

                        break

        theta = atan2(rotated_point.y, rotated_point.x)

        if self.is_look_forward_point:
            self.steering = (
                atan2((2 * self.vehicle_length * sin(theta)), self.lfd) * 180 / pi
            )  # deg
            print(self.steering)
            return self.steering
        else:
            print("no found forward point")
            return None
