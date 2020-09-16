#!/usr/bin/env python
#coding: utf-8
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped, TwistStamped, Quaternion
from tf.transformations import euler_from_quaternion
from serial import Serial
from struct import pack
from sensor_msgs.msg import NavSatFix
#  need install serial first!!
#  sudo apt-get install ros-melodic-serial
#  pip install serial
#  pip install pyserial

class XbeeUAV:
    def __init__(self):
        self.gps = NavSatFix()
        # serial port
        self.ser = Serial("/dev/ttyUSB0",57600,timeout=0.5)
        # data
        self._lon = 0
        self._lat = 0
        self._alt = 0



    def global_pos_cb(self, data):
        self.gps = data
        self._lon = self.gps.longitude
        self._lat = self.gps.latitude
        self._alt = self.gps.altitude


    def send_pose(self):
        rospy.init_node('xbee_pose_send', anonymous=True)
        rospy.loginfo("start send xbee pose control..")
        rate = rospy.Rate(30)

        rospy.Subscriber('/mavros/global_position/global',NavSatFix, self.global_pos_cb)

        rospy.loginfo("start send xbee wait")
        while not rospy.is_shutdown():

            data = pack('>2b4ib',-2,34,self._lon,self._lat,self._alt, (self._lon+self._lat+self._alt), 20)
            print(len(data))
            print(self._lon)
            print(self._lat)
            print(self._alt)

            self.ser.write(data)
            # start send data
            rate.sleep()

if __name__ == '__main__':
    uav = XbeeUAV()
    uav.send_pose()