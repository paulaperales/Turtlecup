#!/usr/bin/python
import cv2
import math
import rospy
from sensor_msgs.msg import Image  
from std_msgs.msg import Bool
import numpy as np    
from cv_bridge import CvBridge

import argparse

# Main function
def callback(img): 
    cv_img = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8') # Needed to work with OpenCV
    frameHSV = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV) # RGB image to HSV
    
    # Size of the image
    height = cv_img.shape[0]
    width = cv_img.shape[1]

    # Binary ball extraction
    # Threshold to detection Red
    redBajo1 = np.array([0, 100, 20], np.uint8)
    redAlto1 = np.array([8, 255, 255], np.uint8)
    redBajo2 = np.array([175, 100, 20], np.uint8)
    redAlto2 = np.array([179, 255, 255], np.uint8)
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(maskRed1, maskRed2)
    
    # Improve detection 
    maskRed = cv2.dilate(maskRed,np.ones((5, 5), np.uint8))

    # Binary field lines extraction
    # Threshold to detection White
    white1 = np.array([135, 135, 135], np.uint8)
    white2 = np.array([138, 138, 138], np.uint8)
    whiteMask = cv2.inRange(cv_img, white1, white2)
    
    # Delete possible bad detections
    endWhite = cv2.erode(whiteMask,np.ones((3, 3), np.uint8))
    
    # Ball radius
    radio = 0
    
    # Line moments to compute its centroid
    mW = cv2.moments(endWhite)
    if mW['m00'] != 0:
        xW = mW['m10']/mW['m00']
        yW = mW['m01']/mW['m00']

    # Ball moments to compute its centroid
    m = cv2.moments(maskRed)
    if m['m00'] != 0:
        x = m['m10']/m['m00']
        y = m['m01']/m['m00']

        # Compute ball area to find its radius        
        _, thresh = cv2.threshold(maskRed, 127, 255, 0) 
        contours,_ = cv2.findContours(thresh, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
        if len(contours) > 0:
            area = cv2.contourArea(contours[0])
            radio = math.sqrt(area/math.pi)
        
        # If the bottom point of the ball overcomes the white line, it's gol
        if y+radio > yW:
            pub_goal.publish(Bool(True))
        else:
            pub_goal.publish(Bool(False))

# Arguments manager
parser = argparse.ArgumentParser(description='Detect goal from VAR robots')
parser.add_argument('-VAR_name', type=str, help='VAR name. Choose between VAR_BLUE and VAR_YELLOW')
ros_args = rospy.myargv()
args = parser.parse_args(ros_args[1:])

bridge = CvBridge()
rospy.init_node('camera_goal') # Node initialization
sub = rospy.Subscriber(ros_args[2]+'/camera/rgb/image_raw', Image, callback) # Subscriber node initialization
pub_goal = rospy.Publisher('/goal', Bool, queue_size=5) # Publisher node initialization

# ROS Loop
rospy.spin()     
