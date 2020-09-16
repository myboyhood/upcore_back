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

class XbeeUAV:
    def __init__(self, id=""):
        # self.uav_id = id
        self.car_pos = PoseStamped()
        self.plane_pos = PoseStamped()
        self.car_gps = NavSatFix()
        self.plane_home_gps = NavSatFix()
        self.start_trace_msg = Bool()
        self.plane_home_gps_static_lon = 0
        self.plane_home_gps_static_lat = 0
        self.get_home_gps = True
        # self.velocity = TwistStamped()
        # port
        # self.ser = Serial("/dev/ttyUSB1", 57600, timeout=0.03)
        self.pub = rospy.Publisher('/car_local_position', PoseStamped, queue_size=10)
        # lon_x lat_y ratio
        self.lon_x = 1.33021075925e-05
        self.lat_y = 8.98488381215e-06
        # data
        self.car_start_x = 0
        self.car_start_y = 0
        self.car_end_x = 20
        self.car_end_y = -5
        self.interval_x = 100.0
        self.interval_y = 100.0
        self._lon = 0
        self._lat = 0
        self._alt = 0
        self._sum = 0
        self.car_pos.pose.position.x = self.car_start_x
        self.car_pos.pose.position.y = self.car_start_y
        self.delta_x = 0.0
        self.delta_y = 0.0


    def plane_home_pos_cb(self,data):
        self.plane_home_gps = data
        if self.get_home_gps and abs(self.plane_home_gps.longitude) > 1:
            self.get_home_gps = False
            self.plane_home_gps_static_lon = self.plane_home_gps.longitude
            self.plane_home_gps_static_lat = self.plane_home_gps.latitude

    def calculate_delta_xy(self):
        self.delta_x = (self.car_end_x - self.car_start_x)/self.interval_x
        self.delta_y = (self.car_end_y - self.car_start_y)/self.interval_y


    def start_trace_cb(self,data):
        self.start_trace_msg = data

    def recv_pose(self):
        rospy.init_node('xbee_pose_recv', anonymous=True)
        rospy.loginfo("start send xbee pose recv..")
        rospy.Subscriber('/mavros/global_position/global', NavSatFix, self.plane_home_pos_cb)
        rospy.Subscriber('/start_trace', Bool, self.start_trace_cb)
        rate = rospy.Rate(5)

        self.calculate_delta_xy()
        # rate.sleep()
        while not rospy.is_shutdown() and not self.start_trace_msg.data:
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
            print("self.delta_x: ", self.delta_x)
            print("self.delta_y: ", self.delta_y)
            if abs(self.car_pos.pose.position.x - self.car_end_x) > 2:
                self.car_pos.pose.position.x += self.delta_x
                self.car_pos.pose.position.y += self.delta_y
            else:
                self.car_pos.pose.position.x = self.car_end_x
                self.car_pos.pose.position.y = self.car_end_y

            """<<<<<<test in gazebo, pub virtual points"""
            print("self.car_pos.pose.position_x", self.car_pos.pose.position.x)
            print("self.car_pos.pose.position_y", self.car_pos.pose.position.y)
            self.pub.publish(self.car_pos)
            rate.sleep()

if __name__ == '__main__':
    uav = XbeeUAV()
    uav.recv_pose()