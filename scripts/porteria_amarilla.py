#!/usr/bin/python
# nodo suscriptor de la camara
import cv2
import rospy
from sensor_msgs.msg import Image  
import numpy as np  
from cv_bridge import CvBridge

# azul1 13 45 71
# azul2 
# amarillo1 103, 100, 22
# amarillo2 127, 125, 27

def callback(img):
    
    cv_img = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')    

    # -------- EN BGR --------- 
    yellow_lower = np.array([20,95,95],np.uint8)
    yellow_upper = np.array([32,141,145],np.uint8)
    mask = cv2.inRange(cv_img, yellow_lower, yellow_upper)

    m = cv2.moments(mask)
    if m['m00'] != 0:
        x = m['m10']/m['m00']
        y = m['m01']/m['m00']
    
        cv2.circle(cv_img, (int(x),int(y)), 5, 255, 5)

    cv2.imshow("Porteria amarilla", cv_img)
    cv2.waitKey(3)


# cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE)
bridge = CvBridge()
rospy.init_node('read_camera')                                                  # Node initialization
sub = rospy.Subscriber('/player1/camera/rgb/image_raw', Image, callback)                            # Subscriber node initialization

rospy.spin()     












