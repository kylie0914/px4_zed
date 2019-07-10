#!/usr/bin/env python

import roslib
import sys
import rospy
import cv2
import numpy as np
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image

class depth_processing:
    def __init__(self):

        rospy.init_node('zed_depth', anonymous=True)
        self.bridge_object = CvBridge()
        rospy.Subscriber("/zed/zed_node/depth/depth_registered",Image,self.depth_callback)
        rospy.Subscriber("/zed/zed_node/left/image_rect_color",Image,self.camera_left_callback)
        rospy.Subscriber("/zed/zed_node/right/image_rect_color",Image,self.camera_right_callback)

    def depth_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image = self.bridge_object.imgmsg_to_cv2(data,desired_encoding='32FC1')
        except CvBridgeError as e:
            print(e)
        #cv2.imshow("depth",cv_image)
        #cv2.waitKey(1)
    def camera_left_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image1 = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        
        left=cv2.cvtColor(cv_image1,cv2.COLOR_BGR2GRAY)
        cv2.imshow("left",cv_image1)
        cv2.waitKey(1)

    def camera_right_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image2 = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
        except CvBridgeError as e:
            print(e)
        
        right=cv2.cvtColor(cv_image2,cv2.COLOR_BGR2GRAY)
        #cv2.imshow("right",right)
        #cv2.waitKey(1)  
