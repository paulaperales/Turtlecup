#!/usr/bin/env python

from cgitb import reset
from doctest import ELLIPSIS_MARKER
from re import M, T
from unittest import result
import rospy
import smach
import cv2
import numpy as np    
from cv_bridge import CvBridge
from sensor_msgs.msg import Image  
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan  
import tf
import math
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import Bool
import argparse

# Arguments manager
parser = argparse.ArgumentParser(description='Create GOALKEEPERS state machine')
parser.add_argument('-GK_name', type=str, help='GK name. Choose between GK_BLUE and GK_YELLOW')
ros_args = rospy.myargv()
args = parser.parse_args(ros_args[1:])

# Define state SearchBall
class SearchBall(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['far', 'near'])

        self.bridge = CvBridge()
        self.sub = rospy.Subscriber(ros_args[2]+'/camera/rgb/image_raw', Image, self.callback)
        self.sub_odom = rospy.Subscriber(ros_args[2]+'/odom', Odometry, self.data_odom)
        self.sub_scan = rospy.Subscriber(ros_args[2]+'/scan', LaserScan, self.data_scan)
        
        self.x = 0
        self.width = 0
        self.rango_medio = 10
        self.result = 0
        self.found = True
        
        self.vector_odom = Odometry().pose.pose
        
        self.theta_robot = 0
        self.last_turn = 0
        
        self.scan = LaserScan().ranges
        
        self.average = math.inf # This must be initialized to infinity because it could be 0 after
        self.range = 1
        self.result = False
        
    def data_scan(self, msg): # Extract laser data
        self.vector_scan = msg.ranges
        average = 0
        c = 0
        size = int(len(self.vector_scan)/2)
        for i in range(-5,5): # Compute an average to be more precise
            if not np.isnan(self.vector_scan[size + i]):
                c = c + 1
                average = average + self.vector_scan[size + i]
        if c != 0:
            self.average = average/c
        else:
            self.average = self.vector_scan[size/2]

    def data_odom(self, msg): # Extract odom data, pose of robot
        self.vector_odom = msg.pose.pose

    def callback(self, img): # Compute ball position 
        cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        frameHSV = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        self.width = cv_img.shape[1]

        # Binarize ball
        # Threshold to detection Red
        redBajo1 = np.array([0, 100, 20], np.uint8)
        redAlto1 = np.array([8, 255, 255], np.uint8)
        redBajo2=np.array([175, 100, 20], np.uint8)
        redAlto2=np.array([179, 255, 255], np.uint8)
        maskRed1 = cv2.inRange(frameHSV, redBajo1, redAlto1)
        maskRed2 = cv2.inRange(frameHSV, redBajo2, redAlto2)
        maskRed = cv2.add(maskRed1, maskRed2)
        
        # Ball moments to compute its centroid
        m = cv2.moments(maskRed)
        if m['m00'] != 0:
            self.found = True # If ball is found
            self.x = m['m10']/m['m00']
        else:
            self.found = False # If ball is not found
            

    def orientate(self): # Orient the robot to place ball in the center
        pub = rospy.Publisher(ros_args[2]+'/mobile_base/commands/velocity', Twist, queue_size=1)
        giro = Twist() # Create the movement

        if self.found:
            if int(self.x) < self.width/2 + self.rango_medio and int(self.x) > self.width/2 - self.rango_medio:
                giro.angular.z = 0 # If the ball is in the center, the robot does not have to move 
            else: # If the ball is located but not in the center, rotate the robot until it is found
                if self.x > self.width/2:
                    giro.angular.z = -0.5
                else:
                    giro.angular.z = 0.5
            if self.average < self.range: # If the ball is clpse to the goal
                self.result = True
            else:
                self.result = False
        else: # If the ball is not found, rotate the robot faster
            self.result = False
            # Robot's orientation
            q = [self.vector_odom.orientation.x, self.vector_odom.orientation.y, self.vector_odom.orientation.z, self.vector_odom.orientation.w] 
            self.theta_robot = tf.transformations.euler_from_quaternion(q)[2]
            # Rotate the robot only in its range (-45º and 45º)
            if self.theta_robot < math.radians(45) and self.last_turn >= 0:
                giro.angular.z = 1
                self.last_turn = 1
            elif self.theta_robot > math.radians(45) and self.last_turn >= 0:
                giro.angular.z = -1
                self.last_turn = -1
            elif self.theta_robot > math.radians(-45) and self.last_turn < 0:
                giro.angular.z = -1
                self.last_turn = -1
            else:
                giro.angular.z = 1
                self.last_turn = 1

        pub.publish(giro)

    def execute(self, userdata):
        self.orientate()
        if self.result: # If the robot has found the ball and it is near
            return 'near'
        else: # If the robot has not found the ball or it is far
            return 'far'


# Define state Catch
class Catch(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gol', 'caught'])
        self.sub_odom = rospy.Subscriber(ros_args[2]+'/odom', Odometry, self.data_odom)
        self.sub_gol = rospy.Subscriber('/goal', Bool, self.is_gol)
        self.pub = rospy.Publisher(ros_args[2]+'/mobile_base/commands/velocity', Twist, queue_size=1)

        self.vector_odom = Odometry().pose.pose
        self.gol = False
        self.result = False


    def data_odom(self, msg): # Extract odom data, pose of robot
        self.vector_odom = msg.pose.pose

    def is_gol(self, msg): # Using the VAR, detect the goal
        if msg == Bool(True):
            self.gol = True
        else:
            self.gol = False
        
    def GoForward(self): # Go forward to stop the ball
        c = 0
        mov = Twist()
        mov.linear.x = 0.5
        while c < 10: # The robot has 1 second to stop it
            self.pub.publish(mov)
            c = c + 1
            rospy.sleep(0.1)
        # Stop the robot
        mov.angular.z = 0
        mov.linear.x = 0
        self.pub.publish(mov)
        
    def execute(self, userdata):
        if self.gol: # If the player has scored a goal
            return 'gol'
        else: # If no goal has been scored
            self.GoForward()
            rospy.sleep(2)
            return 'caught'
            

        

# Define state Reset
class Reset(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gol', 'arrived'])
        
        self.sub_odom = rospy.Subscriber(ros_args[2]+'/odom', Odometry, self.data_odom)
        self.sub_gol = rospy.Subscriber('/goal', Bool, self.is_gol)
        self.pub = rospy.Publisher(ros_args[2]+'/mobile_base/commands/velocity', Twist, queue_size=1)
        
        self.vector_odom = Odometry().pose.pose


    def data_odom(self, msg): # Extract odom data, pose of robot
        self.vector_odom = msg.pose.pose

    def is_gol(self, msg): # Using the VAR, detect the goal
        if msg == Bool(True):
            self.gol = True
        else:
            self.gol = False
    
    def compute_reset(self): # Return the robot to its starting position
        c = 0
        mov = Twist()
        # Perform the reverse movement for 1 second
        mov.linear.x = -0.5
        while c < 10: 
            self.pub.publish(mov)
            c = c + 1
            rospy.sleep(0.1)
        mov.angular.z = 0
        mov.linear.x = 0
        self.pub.publish(mov)

    def execute(self, userdata):
        if self.gol: # If the player has scored a goal 
            return 'gol'
        else: # If the player has not scored a goal
            self.compute_reset()
            rospy.sleep(2)
            return 'arrived'


# Main function
def main():
    
    rospy.init_node('smach_example_state_machine_goalkeeper')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['GOL', 'NO GOL'])
    
    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('SEARCHBALL', SearchBall(), 
                               transitions={'far':'SEARCHBALL', 
                                            'near':'CATCH'})
        
        smach.StateMachine.add('CATCH', Catch(), 
                               transitions={'gol':'GOL', 
                                            'caught':'RESET'})
                                            
        smach.StateMachine.add('RESET', Reset(), 
                               transitions={'gol':'GOL', 
                                            'arrived':'NO GOL'})

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
