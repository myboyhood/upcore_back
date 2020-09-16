#!/usr/bin/env python
#coding: utf-8

import rospy
from mavros_msgs.msg import GlobalPositionTarget, State

from sensor_msgs.msg import NavSatFix
from mavros_msgs.srv import CommandBool, CommandBoolRequest, CommandBoolResponse
from mavros_msgs.srv import SetMode, SetModeRequest, SetModeResponse
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import sys, select, os




class UAVCtl:
    def __init__(self):

        self.home_gps_pub = rospy.Publisher('/plane_home_gps',NavSatFix,queue_size=1)
        self.plane_local_pos_pub = rospy.Publisher('/mavros/setpoint_position/local', PoseStamped, queue_size=1)
        self.start_trace_pub = rospy.Publisher('/start_trace', Bool, queue_size=1)
        self.change_attitude_pub = rospy.Publisher('/change_to_attitude', Bool, queue_size=1)
        self.plane_target_pos = PoseStamped()
        self.plane_curr = PoseStamped()
        self.car_pos = PoseStamped()
        self.init_hover_mode = True
        self.change_attitude = Bool()
        self.follow_mode = Bool()
        self.start_trace_msg = Bool()
        self.fail_safe_mode = False
        self.car_distance_x = 5
        self.init_x = 0
        self.init_y = 0
        self.init_z = 3
        self.destination_x = 0
        self.destination_y = 0
        self.delta_x = 0
        self.delta_y = 0
        self.points_num_x = 0  # need how many points to reach pre-position
        self.points_num_y = 0  # need how many points to reach pre-position
        self.point_x = 0
        self.point_y = 0
        self.step_x = 0
        self.step_y = 0
        self.traj_points_count = 0  # increase count in while loop to pub trajectory points
        self.waypoints_num = 0
        self.waypoints_list_x = list()
        self.waypoints_list_y = list()
        self.plan_enable = True
        self.plan_num = 0


    def state_cb(self, data):
        self.uav_state = data

    def car_local_pos_cb(self, data):
        self.car_pos = data

    def plane_local_pos_cb(self, data):
        self.plane_curr = data

    def change_follow_mode_cb(self, data):
        self.follow_mode.data = data


    def calculate_waypoints(self, start_x, start_y, end_x, end_y):
        self.plan_num += 1
        print("plan_num: ", self.plan_num)
        self.waypoints_list_x = []
        self.waypoints_list_y = []
        self.traj_points_count = 0
        self.points_num_x = math.ceil(abs(self.delta_x)/1)  # how many times of 0.5m
        self.points_num_y = math.ceil(abs(self.delta_y)/1)  # how many times of 0.5m
        print("point_num_x:", self.points_num_x)
        print("point_num_y:", self.points_num_y)
        self.waypoints_num = int(max(self.points_num_x, self.points_num_y))
        self.step_x = (end_x - start_x) / self.waypoints_num
        self.step_y = (end_y - start_y) / self.waypoints_num
        for i in range(self.waypoints_num):
            print("i:", i)
            self.point_x = start_x + (i+1) * self.step_x
            self.point_y = start_y + (i+1) * self.step_y
            self.waypoints_list_x.append(self.point_x)
            self.waypoints_list_y.append(self.point_y)
            print("list_x: ", self.waypoints_list_x)
            print("list_y: ", self.waypoints_list_y)





    def ros_node(self):
        rospy.init_node('gps_pos_ctl')
        rate = rospy.Rate(20)
        rospy.Subscriber('/car_local_position', PoseStamped, self.car_local_pos_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.plane_local_pos_cb)
        rospy.Subscriber('/change_to_follow_topic', Bool, self.change_follow_mode_cb)




        self.start_trace_msg.data = False
        while (not rospy.is_shutdown()) and self.init_hover_mode:
            self.plane_target_pos.pose.position.x = self.init_x
            self.plane_target_pos.pose.position.y = self.init_y
            self.plane_target_pos.pose.position.z = self.init_z
            self.plane_local_pos_pub.publish(self.plane_target_pos)
            rate.sleep()
            print("hover mode: ", self.init_hover_mode)
            print("curr_local_position x, y, z\n")
            print(self.plane_curr.pose.position.x)
            print(self.plane_curr.pose.position.y)
            print(self.plane_curr.pose.position.z)
            if abs(self.plane_curr.pose.position.x - self.init_x) < 0.2 and abs(self.plane_curr.pose.position.y - self.init_y) < 0.2 and abs(self.plane_curr.pose.position.z - self.init_z) < 0.2:
                self.init_hover_mode = False
                self.follow_mode.data = True
                self.start_trace_msg.data = True

        """
        into follow mode
        """
        while (not rospy.is_shutdown()) and self.follow_mode:
            self.start_trace_pub.publish(self.start_trace_msg)
            """decide follow orientation"""
            if self.car_pos.pose.position.x > self.plane_curr.pose.position.x:
                forward_follow = 1
            else:
                forward_follow = -1

            self.destination_x = self.car_pos.pose.position.x - forward_follow * self.car_distance_x
            self.destination_y = self.car_pos.pose.position.y
            print("car_pos_x:", self.car_pos.pose.position.x)
            print("car_pos_y:", self.car_pos.pose.position.y)
            print("des_x:", self.destination_x)
            print("des_y:", self.destination_y)

            # """ trace mode """
            # # the distance between curr_position and destination
            # self.delta_x = self.destination_x - self.plane_curr.pose.position.x
            # self.delta_y = self.destination_y - self.plane_curr.pose.position.y
            # if abs(self.delta_x) > 0.5 or abs(self.delta_y) > 0.5:
            #     print("big error ")
            #     # plan every time, calculate every time
            #     self.calculate_waypoints(start_x=self.plane_curr.pose.position.x, start_y=self.plane_curr.pose.position.y,end_x=self.destination_x,end_y=self.destination_y)
            #
            #     """whether reached the trajectory point or not"""
            #     if abs(self.waypoints_list_x[self.traj_points_count] - self.plane_curr.pose.position.x) > 0.2 or abs(self.waypoints_list_y[self.traj_points_count] - self.plane_curr.pose.position.y) > 0.2:
            #         print("reaching the {} trajectory point".format(self.traj_points_count))
            #         print("x_err between traj_point and curr: ", self.waypoints_list_x[self.traj_points_count] - self.plane_curr.pose.position.x)
            #         print("y_err between traj_point and curr: ", self.waypoints_list_y[self.traj_points_count] - self.plane_curr.pose.position.y)
            #         self.plane_target_pos.pose.position.x = self.waypoints_list_x[self.traj_points_count]
            #         self.plane_target_pos.pose.position.y = self.waypoints_list_y[self.traj_points_count]
            #     else:
            #         if self.traj_points_count < len(self.waypoints_list_x) - 1:
            #             self.traj_points_count += 1  # next point
            #         self.plane_target_pos.pose.position.x = self.waypoints_list_x[self.traj_points_count]
            #         self.plane_target_pos.pose.position.y = self.waypoints_list_y[self.traj_points_count]
            # else:
            #     self.plane_target_pos.pose.position.x = self.destination_x
            #     self.plane_target_pos.pose.position.y = self.destination_y
            self.plane_target_pos.pose.position.x = self.destination_x
            self.plane_target_pos.pose.position.y = self.destination_y

            self.plane_target_pos.pose.position.z = self.init_z
            self.plane_local_pos_pub.publish(self.plane_target_pos)
            print("self.plane_target_pos.pose.position.x: ", self.plane_target_pos.pose.position.x)
            print("self.plane_target_pos.pose.position.y: ", self.plane_target_pos.pose.position.y)
            print("self.plane_target_pos.pose.position.z: ", self.plane_target_pos.pose.position.z)
            print("curr_x: ", self.plane_curr.pose.position.x)
            print("curr_y: ", self.plane_curr.pose.position.y)
            print("curr_z: ", self.plane_curr.pose.position.z)
            rate.sleep()


if __name__ == '__main__':
    uav = UAVCtl()
    uav.ros_node()




