#!/usr/bin/env python

import rospy
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError


path = "/home/sun/sdx_ws/realsense_ws/src/realsense-ros/realsense2_camera/images/"
counter = 0

def callback(img_ori):    
    bridge = CvBridge()
    try:
        img = bridge.imgmsg_to_cv2(img_ori, "bgr8")
    except CvBridgeError, e1:
        print(e1)
        return

    cv2.imshow("capture", img)
    if cv2.waitKey(10) & 0xFF == ord('c'):
        global counter, path
        cv2.imwrite(path + str(counter) + ".png", img)
        counter += 1
        print("image "+str(counter) + "saved in " + path)
  
if __name__ == '__main__':
    rospy.init_node('image_collector', anonymous=True)
    rospy.Subscriber("/camera/color/image_raw", Image, callback)
    rospy.spin()
