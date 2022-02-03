#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
import sys
from math import pi

import rospy
import tf
from morai_msgs.msg import EgoVehicleStatus, ObjectStatusList
from nav_msgs.msg import Path
from std_msgs.msg import Float64

from lib.utils import (
    CruiseControl,
    PathReader,
    PIDController,
    PurePursuit,
    ValidObject,
    VelocityPlanning,
    findLocalPath,
    latticePlanner,
)


class WecarPlanner:
    STEERING_ANGLE_TO_SERVO_OFFSET = 0.5304  ## servo motor offset
    RPM_GAIN = 4616
    RATE = 30

    def __init__(self):
        rospy.init_node("wecar_planner", anonymous=True)

        arg = rospy.myargv(argv=sys.argv)
        self.path_name = arg[1]

        # publisher
        global_path_pub = rospy.Publisher(
            "/global_path", Path, queue_size=1
        )  ## global_path publisher
        local_path_pub = rospy.Publisher(
            "/local_path", Path, queue_size=1
        )  ## local_path publisher

        self.motor_pub = rospy.Publisher("commands/motor/speed", Float64, queue_size=1)
        self.servo_pub = rospy.Publisher(
            "commands/servo/position", Float64, queue_size=1
        )
        ########################  lattice  ########################
        latticePathCount = 7
        self.latticePathPub = []
        for i in range(latticePathCount):
            self.latticePathPub.append(
                rospy.Publisher(f"lattice_path_{i + 1}", Path, queue_size=1)
            )
        ########################  lattice  ########################

        # subscriber
        rospy.Subscriber(
            "/Ego_topic", EgoVehicleStatus, self.statusCB
        )  ## Vehicle Status Subscriber
        rospy.Subscriber(
            "/Object_topic", ObjectStatusList, self.objectInfoCB
        )  ## Object information Subscriber

        # def
        self.is_status = False  ## 차량 상태 점검
        self.is_obj = False  ## 장애물 상태 점검

        # msg
        self.motor_msg = Float64()
        self.servo_msg = Float64()

        # class
        self.purePursuit = PurePursuit()  ## purePursuit import
        self.cruiseControl = CruiseControl(
            0.5, 1
        )  ## cruiseControl import (object_vel_gain, object_dis_gain)
        self.validObject = ValidObject()  ## 장애물 유무 확인 ()
        pid = PIDController()  ## pidController import

        # read path
        self.global_path = PathReader("wecar_ros").loadPath(f"{self.path_name}.txt")

        # time var
        count = 0
        rate = rospy.Rate(self.RATE)

        latticeCurrentLane = 3

        while not rospy.is_shutdown():
            print(self.is_status, self.is_obj)

            if self.is_status == True and self.is_obj == True:  ## 차량의 상태, 장애물 상태 점검
                ## global_path와 차량의 status_msg를 이용해 현제 waypoint와 local_path를 생성
                self.localPath, self.current_waypoint = findLocalPath(
                    self.global_path, self.status_msg
                )

                ## 장애물의 숫자와 Type 위치 속도 (object_num, object type, object pose_x, object pose_y, object velocity)
                self.validObject.getObject(
                    self.object_num,
                    self.object_info[0],
                    self.object_info[1],
                    self.object_info[2],
                    self.object_info[3],
                )

                globalObj, localObj = self.validObject.calcValidObj(
                    x=self.status_msg.position.x,
                    y=self.status_msg.position.y,
                    heading=(self.status_msg.heading) / 180 * pi,
                )

                ########################  lattice  ########################
                vehicleStatus = [
                    self.status_msg.position.x,
                    self.status_msg.position.y,
                    (self.status_msg.heading) / 180 * pi,
                    self.status_msg.velocity.x / 3.6,
                ]
                latticePath, selectedLane = latticePlanner(
                    self.localPath, globalObj, vehicleStatus, latticeCurrentLane
                )
                latticeCurrentLane = selectedLane

                if selectedLane != -1:
                    self.localPath = latticePath[selectedLane]

                if len(latticePath) == latticePathCount:
                    for i in range(latticePathCount):
                        self.latticePathPub[i].publish(latticePath[i])
                ########################  lattice  ########################

                self.cruiseControl.checkObject(self.localPath, globalObj, localObj)

                # Set msg & publish
                self.setServoMsg()
                self.setMotorMsg(localObj)
                self.servo_pub.publish(self.servo_msg)
                self.motor_pub.publish(self.motor_msg)

                # Print Local Path to RViz
                local_path_pub.publish(self.localPath)  ## Local Path 출력
                # Print Info
                self.log()

            # Global Path 출력 (버그 때문에 일정 count 이후에 출력됨)
            if count == 300:  ## global path 출력
                global_path_pub.publish(self.global_path)
                count = 0
            count += 1

            rate.sleep()

    def setServoMsg(self):
        self.purePursuit.getPath(self.localPath)  ## pure_pursuit 알고리즘에 Local path 적용
        self.purePursuit.getEgoStatus(
            self.status_msg
        )  ## pure_pursuit 알고리즘에 차량의 status 적용
        self.steering = self.purePursuit.steering_angle()
        self.servo_msg = self.steering * 0.021 + self.STEERING_ANGLE_TO_SERVO_OFFSET

    def setMotorMsg(self, localObj):
        velProfile = VelocityPlanning(10, 0.15).curveBasedVelocity(self.global_path, 30)
        self.cc_vel = self.cruiseControl.acc(
            localObj,
            self.status_msg.velocity.x,
            velProfile[self.current_waypoint],
            self.status_msg,
        )  ## advanced cruise control 적용한 속도 계획
        self.motor_msg = self.cc_vel * self.RPM_GAIN / 3.6

    def log(self):

        os.system("clear")
        print("--------------------status-------------------------")
        print(
            f"position :{self.status_msg.position.x} ,{self.status_msg.position.y}, {self.status_msg.position.z}"
        )
        print(f"velocity :{self.status_msg.velocity.x} km/h")
        print(f"heading :{self.status_msg.heading} deg")

        print("--------------------object-------------------------")
        print(f"object num :{self.object_num}")
        for i in range(0, self.object_num):
            print(
                f"{i} : type = {self.object_info[0]}, x = {self.object_info[1]}, y = {self.object_info[2]}, z = {self.object_info[3]} "
            )

        print("--------------------controller-------------------------")
        print(f"target vel_planning :{self.cc_vel} km/h")
        print(f"target steering_angle :{self.steering} deg")

        print("--------------------localization-------------------------")
        print(f"all waypoint size: {len(self.global_path.poses)} ")
        print(f"current waypoint : {self.current_waypoint} ")

    def statusCB(self, data):  ## Vehicle Status Subscriber
        self.status_msg = data
        br = tf.TransformBroadcaster()
        br.sendTransform(
            (
                self.status_msg.position.x,
                self.status_msg.position.y,
                self.status_msg.position.z,
            ),
            tf.transformations.quaternion_from_euler(
                0, 0, (self.status_msg.heading) / 180 * pi
            ),
            rospy.Time.now(),
            "gps",
            "map",
        )
        self.is_status = True

        # print(self.status_msg.yaw)

    def objectInfoCB(self, data):  ## Object information Subscriber
        self.object_num = (
            data.num_of_npcs + data.num_of_obstacle + data.num_of_pedestrian
        )
        object_type = []
        object_pose_x = []
        object_pose_y = []
        object_velocity = []
        for num in range(data.num_of_npcs):
            object_type.append(data.npc_list[num].type)
            object_pose_x.append(data.npc_list[num].position.x)
            object_pose_y.append(data.npc_list[num].position.y)
            object_velocity.append(data.npc_list[num].velocity.x)

        for num in range(data.num_of_obstacle):
            object_type.append(data.obstacle_list[num].type)
            object_pose_x.append(data.obstacle_list[num].position.x)
            object_pose_y.append(data.obstacle_list[num].position.y)
            object_velocity.append(data.obstacle_list[num].velocity.x)

        for num in range(data.num_of_pedestrian):
            object_type.append(data.pedestrian_list[num].type)
            object_pose_x.append(data.pedestrian_list[num].position.x)
            object_pose_y.append(data.pedestrian_list[num].position.y)
            object_velocity.append(data.pedestrian_list[num].velocity.x)

        self.object_info = [object_type, object_pose_x, object_pose_y, object_velocity]
        self.is_obj = True


if __name__ == "__main__":
    try:
        kcity_pathtracking = WecarPlanner()
    except rospy.ROSInterruptException:
        pass
