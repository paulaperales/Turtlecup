#!/usr/bin/python
# nodo suscriptor de la camara
import cv2
import rospy
from sensor_msgs.msg import Image  
from std_msgs.msg import String
import numpy as np    
from cv_bridge import CvBridge


def callback(img):
    
    
    cv_img = bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
    frameHSV = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

    height = cv_img.shape[0]
    width = cv_img.shape[1]

    rango_medio = 10

    # Extracción de la pelota en binario
    redBajo1 = np.array([0, 100, 20], np.uint8)
    redAlto1 = np.array([8, 255, 255], np.uint8)
    redBajo2=np.array([175, 100, 20], np.uint8)
    redAlto2=np.array([179, 255, 255], np.uint8)
    maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
    maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
    maskRed = cv2.add(maskRed1, maskRed2)

    # Cálculo del centroide en la imagen binaria
    m = cv2.moments(maskRed)
    if m['m00'] != 0:
        x = m['m10']/m['m00']
        y = m['m01']/m['m00']


        if int(x) < width/2 + rango_medio and int(x) > width/2 - rango_medio:
            pub_found.publish(String('centered'))
        else:
            pub_found.publish(String('notcentered'))

        cv2.circle(cv_img, (int(x),int(y)), 5, 255, 5)
    else:
        pub_found.publish(String('notfound'))


    


    cv2.imshow("Image window", cv_img)
    cv2.imshow("Pelota", maskRed)
    cv2.waitKey(3)


# cv2.namedWindow('Camera', cv2.WINDOW_AUTOSIZE)
bridge = CvBridge()
rospy.init_node('read_camera')                                                                      # Node initialization
sub = rospy.Subscriber('/player1/camera/rgb/image_raw', Image, callback)                            # Subscriber node initialization
pub_found = rospy.Publisher('/ball_is_found', String, queue_size=5)

rospy.spin()     












