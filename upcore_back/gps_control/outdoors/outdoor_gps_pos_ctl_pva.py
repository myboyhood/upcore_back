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
        self.change_attitude_pub = rospy.Publisher('/change_to_attitude', Bool, queue_size=1)
        self.plane_target_pos = PoseStamped()
        self.plane_curr = PoseStamped()
        self.car_pos = PoseStamped()
        self.init_hover_mode = True
        self.change_attitude = Bool()
        self.follow_mode = Bool()
        self.fail_safe_mode = False
        self.car_distance_x = 0
        self.init_count = 100 # 100个初始位置点，用于起飞悬停
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
        rospy.Subscriber('/car_local_positoin', PoseStamped, self.car_local_pos_cb)
        rospy.Subscriber('/mavros/local_position/pose', PoseStamped, self.plane_local_pos_cb)
        rospy.Subscriber('/change_to_follow_topic', Bool, self.change_follow_mode_cb)

        """
        test outdoor
        """
        while not rospy.is_shutdown() and self.init_count > 0:
            self.init_x += self.plane_curr.pose.position.x
            self.init_y += self.plane_curr.pose.position.y
            self.init_count -= 1
            rospy.loginfo("acquire init xy , count = %d", self.init_count)
            rate.sleep()

        self.init_x /= 100.0
        self.init_y /= 100.0
        # if abs(self.init_x) > 3 or abs(self.init_y) > 3:
        #     while not rospy.is_shutdown():
        #         print("the error of init xy is too large, error_x = %f, error_y = %f", self.init_x, self.init_y)
        #         rate.sleep()

        local_pub_pos = PoseStamped()
        local_pub_pos.pose.position.x = self.init_x/3.0
        local_pub_pos.pose.position.y = self.init_y/3.0
        local_pub_pos.pose.position.z = self.init_z/3.0

        count = 0
        while not (rospy.is_shutdown() or count > 150):
            self.plane_local_pos_pub.publish(local_pub_pos)
            print("x: ",local_pub_pos.pose.position.x)
            print("y: ",local_pub_pos.pose.position.y)
            print("z: ",local_pub_pos.pose.position.z)
            print("pub local position : %d ", count)
            rate.sleep()
            count += 1

        # self.uav_state.mode = "STABILIZED"
        # while (not rospy.is_shutdown()) and (self.uav_state.mode != "OFFBOARD"):
        #     self.plane_local_pos_pub.publish(local_pub_pos)
        #     rospy.loginfo("wait for OFFBOARD switch")
        #     rate.sleep()

        """
        reach hover point
        """
        """first height"""
        # local_pub_pos = PoseStamped()
        local_pub_pos.pose.position.x = self.init_x/3.0
        local_pub_pos.pose.position.y = self.init_y/3.0
        local_pub_pos.pose.position.z = self.init_z/3.0

        count = 0
        while not (rospy.is_shutdown() or count > 50):
            self.plane_local_pos_pub.publish(local_pub_pos)
            print("pub local position : 1m height ")
            rate.sleep()
            count += 1

        """second height"""
        local_pub_pos.pose.position.x = 2.0*self.init_x/3.0
        local_pub_pos.pose.position.y = 2.0*self.init_y/3.0
        local_pub_pos.pose.position.z = 2.0*self.init_z/3.0

        count = 0
        while not (rospy.is_shutdown() or count > 50):
            self.plane_local_pos_pub.publish(local_pub_pos)
            print("pub local position : 2m height ")
            rate.sleep()
            count += 1

        """third height"""
        local_pub_pos.pose.position.x = self.init_x
        local_pub_pos.pose.position.y = self.init_y
        local_pub_pos.pose.position.z = self.init_z

        count = 0
        while not (rospy.is_shutdown() or count > 50):
            self.plane_local_pos_pub.publish(local_pub_pos)
            print("pub local position : 3m height ")
            rate.sleep()
            count += 1

        """
        test outdoor
        """

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
                self.follow_mode.data = False

        """
        into attitude mode
        """
        self.change_attitude.data = True
        self.follow_mode.data = False
        while (not rospy.is_shutdown() and (not self.follow_mode.data)):
            self.change_attitude_pub.publish(self.change_attitude)
            print("change into attitude mode")
            rate.sleep()

        """
        into follow mode
        """
        while (not rospy.is_shutdown()) and self.follow_mode:

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

            """ trace mode """
            # the distance between curr_position and destination
            self.delta_x = self.destination_x - self.plane_curr.pose.position.x
            self.delta_y = self.destination_y - self.plane_curr.pose.position.y
            if abs(self.delta_x) > 0.5 or abs(self.delta_y) > 0.5:
                print("big error ")
                # plan every time, calculate every time
                self.calculate_waypoints(start_x=self.plane_curr.pose.position.x, start_y=self.plane_curr.pose.position.y,end_x=self.destination_x,end_y=self.destination_y)

                """whether reached the trajectory point or not"""
                if abs(self.waypoints_list_x[self.traj_points_count] - self.plane_curr.pose.position.x) > 0.2 or abs(self.waypoints_list_y[self.traj_points_count] - self.plane_curr.pose.position.y) > 0.2:
                    print("reaching the {} trajectory point".format(self.traj_points_count))
                    print("x_err between traj_point and curr: ", self.waypoints_list_x[self.traj_points_count] - self.plane_curr.pose.position.x)
                    print("y_err between traj_point and curr: ", self.waypoints_list_y[self.traj_points_count] - self.plane_curr.pose.position.y)
                    self.plane_target_pos.pose.position.x = self.waypoints_list_x[self.traj_points_count]
                    self.plane_target_pos.pose.position.y = self.waypoints_list_y[self.traj_points_count]
                else:
                    if self.traj_points_count < len(self.waypoints_list_x) - 1:
                        self.traj_points_count += 1  # next point
                    self.plane_target_pos.pose.position.x = self.waypoints_list_x[self.traj_points_count]
                    self.plane_target_pos.pose.position.y = self.waypoints_list_y[self.traj_points_count]
            else:
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




