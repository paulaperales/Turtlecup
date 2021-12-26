# nodo suscriptor de la camara
import cv2
import rospy
from sensor_msgs.msg import Image  
import numpy as np  
from cv_bridge import CvBridge


def callback(img):
    
    cv_img = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')

    cv2.imshow("Image window", cv_img)
    cv2.waitKey(3)


# cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE)
bridge = CvBridge()
rospy.init_node('read_camera')                                                  # Node initialization
sub = rospy.Subscriber('/player1/camera/rgb/image_raw', Image, callback)                            # Subscriber node initialization

rospy.spin()     












