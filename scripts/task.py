#!/usr/bin/python
import rospy
from geometry_msgs.msg import Twist  
from std_msgs.msg import String
from sensor_msgs.msg import LaserScan 
from nav_msgs.msg import Odometry
import tf
import math
import numpy as np
    
vector_scan = LaserScan().ranges
pose_odom = Odometry().pose.pose

def data_scan(msg):
    global vector_scan
    vector_scan = msg.ranges

def data_odom(msg):
    global pose_odom
    pose_odom = msg.pose.pose

def compute_average():
    global vector_scan
    # distance = 10
    # center = int(len(vector_scan)/2-10)
    # sum = 0
    # total = 0
    # if(len(vector_scan) != 0):
    #     for i in range(0,20):
    #         if(vector_scan[i+center] != np.isnan):
    #             sum = sum + vector_scan[i + center]
    #             total = total + 1
    # if(total > 0):
    #     return sum/total
    # else:
    #     return 0
    return vector_scan[int(len(vector_scan)/2)]

def compute_ball_position():    
    global pose_odom
    average = compute_average()   
    pose_ball = Odometry().pose.pose
    q = [pose_odom.orientation.x, pose_odom.orientation.y, pose_odom.orientation.z, pose_odom.orientation.w]    # Definition of the quaternion
    theta = tf.transformations.euler_from_quaternion(q)[2] # Robot's orientation
    if theta < math.pi/2:                                           # First quadrant
        newX = average*math.cos(theta)
        newY = average*math.sin(theta)
    elif theta < math.pi:                                           # Second quadrant
        newX = -average*math.cos((math.pi-theta))
        newY = average*math.sin((math.pi-theta))
    elif theta < 3*math.pi/2:                                       # Third quadrant
        newX = -average*math.cos((math.pi+theta))
        newY = -average*math.sin((math.pi+theta))
    else:                                                                  # Fourth quadrant
        newX = average*math.cos((2*math.pi-theta))
        newY = -average*math.sin((2*math.pi-theta))
    pose_ball.position.x = newX
    pose_ball.position.y = newY
    return pose_ball

def girar_turtlebot(msg):
    giro = Twist()
    if (msg == String('notfound')):
        giro.angular.z = 0.5
        pub.publish(giro)
    elif(msg == String('centered')):   
        giro.angular.z = 0.0
        pub.publish(giro)
        pose_ball = compute_ball_position()
        print(pose_ball)
        
    else:
        giro.angular.z = 0.1
        pub.publish(giro) 
    




rospy.init_node('task')                                                                      # Node initialization
sub = rospy.Subscriber('/ball_is_found', String, girar_turtlebot)                            # Subscriber node initialization
pub = rospy.Publisher('/player1/mobile_base/commands/velocity', Twist, queue_size=5)
sub_scan = rospy.Subscriber('/player1/scan', LaserScan, data_scan)
sub_odom = rospy.Subscriber('/player1/odom', Odometry, data_odom)

rospy.spin() 