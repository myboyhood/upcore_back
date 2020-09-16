#!/usr/bin/env python
#coding: utf-8
import rospy
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from tf.transformations import euler_from_quaternion
from serial import Serial
from struct import pack,unpack
from sensor_msgs.msg import NavSatFix
#  need install serial first!!
#  sudo apt-get install ros-melodic-serial
import math


class XbeeUAV:
    def __init__(self, id=""):
        # self.uav_id = id
        self.car_pos = PoseStamped()
        self.plane_pos = PoseStamped()
        self.car_gps = NavSatFix()
        self.plane_home_gps = NavSatFix()
        self.start_velocity_esti_msg = Bool()
        self.plane_home_gps_static_lon = 0
        self.plane_home_gps_static_lat = 0
        self.get_home_gps = True
        # self.velocity = TwistStamped()
        # port
        # self.ser = Serial("/dev/ttyUSB1", 57600, timeout=0.03)
        self.car_pub = rospy.Publisher('/car_local_position', PoseStamped, queue_size=1)
        self.start_trace_pub = rospy.Publisher('/start_trace_topic', Bool, queue_size=1)
        # lon_x lat_y ratio
        self.lon_x = 1.33021075925e-05
        self.lat_y = 8.98488381215e-06
        # data
        self._lon = 0
        self._lat = 0
        self._alt = 0
        self._sum = 0
        self.car_pos.pose.position.x = -1
        self.car_pos.pose.position.y = 5
        self.yaw_bias = 0.5
        self.car_vel_esti_start_x = self.car_pos.pose.position.x - 20*math.cos(self.yaw_bias)
        self.car_vel_esti_start_y = self.car_pos.pose.position.y - 20*math.sin(self.yaw_bias)
        self.car_vel = 0.6 # 0.6*looprate = velocity
        self.delta_x = self.car_vel*math.cos(self.yaw_bias)
        self.delta_y = self.car_vel*math.sin(self.yaw_bias)


    def plane_home_pos_cb(self,data):
        self.plane_home_gps = data
        if self.get_home_gps and abs(self.plane_home_gps.longitude) > 1:
            self.get_home_gps = False
            self.plane_home_gps_static_lon = self.plane_home_gps.longitude
            self.plane_home_gps_static_lat = self.plane_home_gps.latitude


    def start_velocity_esti_cb(self,data):
        self.start_velocity_esti_msg = data

    def recv_pose(self):
        rospy.init_node('xbee_pose_recv', anonymous=True)
        rospy.loginfo("start send xbee pose recv..")
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.plane_home_pos_cb)
        rospy.Subscriber('/start_velocity_esti_topic', Bool, self.start_velocity_esti_cb)
        rate = rospy.Rate(5)
        # rate.sleep()
        while not rospy.is_shutdown() and not self.start_velocity_esti_msg.data:
            print("wait drone hover...")
            rate.sleep()

        while not rospy.is_shutdown():
            # data = self.ser.read(19)
            # print(len(data))
            # if data != "":
            #     head1,head2,self._lon,self._lat,self._alt,self._sum,end = unpack('>2b4ib', data)
            #     print(self._lon,self._lat,self._alt)
            #
            #     self.car_gps.longitude = self._lon
            #     self.car_gps.latitude = self._lat
            #     self.car_gps.altitude = self._alt
            #     """ change to local position , plane home is origin, x axis towards east, y axis towards west"""
            #     self.car_pos.pose.position.x = (self.car_gps.longitude - self.plane_home_gps_static_lon) / self.lon_x
            #     self.car_pos.pose.position.y = (self.car_gps.latitude - self.plane_home_gps_static_lat) / self.lat_y

            """>>>>>>test in gazebo, pub virtual points"""
            rospy.loginfo("delta_x: %f",self.delta_x)
            rospy.loginfo("delta_y: %f",self.delta_y)

            self.car_vel_esti_start_x += self.delta_x
            self.car_vel_esti_start_y += self.delta_y
            self.car_pos.pose.position.x = self.car_vel_esti_start_x
            self.car_pos.pose.position.y = self.car_vel_esti_start_y
            self.car_pos.pose.position.z = 3 # the height of car is 3m

            """<<<<<<test in gazebo, pub virtual points"""
            print("self.car_pos.pose.position_x", self.car_pos.pose.position.x)
            print("self.car_pos.pose.position_y", self.car_pos.pose.position.y)
            self.car_pub.publish(self.car_pos)
            rate.sleep()

if __name__ == '__main__':
    uav = XbeeUAV()
    uav.recv_pose()