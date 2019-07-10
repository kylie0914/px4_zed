#! /usr/bin/python
import roslib
import sys
import rospy
import cv2
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import numpy as np
import time
#from zed_subs_class import *

class depth_processing:
    def __init__(self):

        self.left_img = None
        self.right_img = None
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
            self.left_img=cv_image1
        except CvBridgeError as e:
            print(e)
        
        left=cv2.cvtColor(cv_image1,cv2.COLOR_BGR2GRAY)


    def camera_right_callback(self,data):
        
        try:
            # We select bgr8 because its the OpneCV encoding by default
            cv_image2 = self.bridge_object.imgmsg_to_cv2(data, desired_encoding='bgr8')
            self.right_img=cv_image2
        except CvBridgeError as e:
            print(e)
        
        right=cv2.cvtColor(cv_image2,cv2.COLOR_BGR2GRAY)
        #cv2.imshow("right",right)
        #cv2.waitKey(1)  

if __name__ == '__main__': 
		try:
				detector = depth_processing()
				frame_rate = rospy.Rate(10)
				while True:						
						print("@@@@@@@@  zed_depth_test @@@@@@@")
						frame_rate.sleep()
						imgL = detector.left_img  
						imgR = detector.right_img
				
						# SGBM Parameters -----------------
						window_size = 3                     # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
						 
						left_matcher = cv2.StereoSGBM_create(
								minDisparity=0,
								numDisparities=160,             # max_disp has to be dividable by 16 f. E. HH 192, 256
								blockSize=5,
								P1=8 * 3 * window_size ** 2,    # wsize default 3; 5; 7 for SGBM reduced size image; 15 for SGBM full size image (1300px and above); 5 Works nicely
								P2=32 * 3 * window_size ** 2,
								disp12MaxDiff=1,
								uniquenessRatio=15,
								speckleWindowSize=0,
								speckleRange=2,
								preFilterCap=63,
								mode=cv2.STEREO_SGBM_MODE_SGBM_3WAY
						)

						right_matcher = cv2.ximgproc.createRightMatcher(left_matcher)
			
						# FILTER Parameters
						lmbda = 80000
						sigma = 1.2
						visual_multiplier = 1.0
						 
						wls_filter = cv2.ximgproc.createDisparityWLSFilter(matcher_left=left_matcher)
						wls_filter.setLambda(lmbda)
						wls_filter.setSigmaColor(sigma)

						print('computing disparity...')
						displ = left_matcher.compute(imgL, imgR)  # .astype(np.float32)/16
						print(1)
						dispr = right_matcher.compute(imgR, imgL)  # .astype(np.float32)/16
						print(2)
						displ = np.int16(displ)
						dispr = np.int16(dispr)
						filteredImg = wls_filter.filter(displ, imgL, None, dispr)  # important to put "imgL" here!!!

						filteredImg = cv2.normalize(src=filteredImg, dst=filteredImg, beta=0, alpha=255, norm_type=cv2.NORM_MINMAX);
						filteredImg = np.uint8(filteredImg)

						cv2.imshow('Disparity Map', filteredImg)

						k = cv2.waitKey(1)					
						if k == 27:   # esc key
								cv2.destroyAllWindow()	
								break



		except rospy.ROSInterruptException:
				rospy.loginfo("Detector node terminated.")
				



'''
						

'''

