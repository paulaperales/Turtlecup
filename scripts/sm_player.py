#!/usr/bin/env python
from cgitb import reset
from doctest import ELLIPSIS_MARKER
from glob import glob
from re import M, T
from tracemalloc import start
from turtle import pu
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
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import Bool
from std_msgs.msg import String
from gazebo_msgs.srv import SpawnModel
from geometry_msgs.msg import Pose, Point, Quaternion

# Ceate a class to move the robot to a desired Point
class ClienteMoveBase:
    def __init__(self):
        # Create a ROS client for the action, needing the node name and the Python class that implement the action
        # To move the robot, these values are "move_base" and "MoveBaseAction"
        self.client =  actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        # Wait until the node is active
        self.client.wait_for_server()

    def moveTo(self, x, y, orientation):
        # A MoveBaseGoal is an objective point to which we want to move
        goal = MoveBaseGoal()
        # Reference system we are using
        goal.target_pose.header.frame_id = "map"
        # Goal point
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        # The orientation is a quaternion
        q = tf.transformations.quaternion_from_euler(0,0,orientation)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        # Send the goal 
        self.client.send_goal(goal)
        # Check from time to time whether the goal has been met
        # Obtaint the action result
        state = self.client.get_state()
        cont = 0
        # ACTIVE is that it is in execution, PENDING is that it has not started yet
        # If it takes more than 10 seconds to arrive, it is a bad result
        while (state==GoalStatus.ACTIVE or state==GoalStatus.PENDING) and cont < 100:
            state = self.client.get_state()
            cont = cont + 1
            rospy.sleep(0.1)
        return self.client.get_result()


# A global value to set the goal point
x = 0


# Define class SpawnBall to place the ball on the field
class SpawnBall:
    def __init__(self):
        rospy.wait_for_service("gazebo/spawn_urdf_model")
        print("Got it.")
        
        # Spawn the ball in the scene
        self.spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
            
        # Open the ball file 
        # **********************IMPORTANT**********************
        # CHANGE THE PATH TO YOUR OWN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
        with open("/home/paula/tb2_ws/src/turtlebot2/turtlebot_simulator/turtlebot_gazebo/Turtlecup/models/ball.urdf.xacro", "r") as f:
            self.ball = f.read()
        self.x = 0
        self.y = 0 

    def spawn_ball(self):
        # Calculate a random position to place the ball
        # It must be between the field limits
        self.x = np.random.choice([1.19211, -1.19211], size=1, p=[0.5,0.5]) # It will apear on one of the two penalty points
        s = np.random.uniform(-0.7, 0.7, 100)
        self.y = np.random.choice(s, size=1)
        item_pose = Pose(position=Point(x=self.x, y=self.y, z=0), orientation=Quaternion(0,0,0,0))
        
        self.spawn_model("ball_description", self.ball, "", item_pose, "world")


# Define state Ball
class Ball(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['centered', 'notcentered'])
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        self.x = 0
        self.width = 0
        self.rango_medio = 10
        self.result = 0
        self.found = True

    def callback(self, img):
        cv_img = self.bridge.imgmsg_to_cv2(img, desired_encoding='bgr8')
        frameHSV = cv2.cvtColor(cv_img, cv2.COLOR_BGR2HSV)

        self.width = cv_img.shape[1]

        # Binary ball extraction
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
            self.found = True
            self.x = m['m10']/m['m00']
        else:
            self.found = False
            

    def orientate(self): # Orient the robot to place ball in the center
        pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        giro = Twist() # Create the movement

        if self.found:
            if int(self.x) < self.width/2 + self.rango_medio and int(self.x) > self.width/2 - self.rango_medio:
                giro.angular.z = 0 # If the ball is in the center, the robot does not have to move 
                self.result = 1
            else: # If the ball is located but not in the center, rotate the robot until it is found
                if self.x > self.width/2:
                    giro.angular.z = -0.5
                    self.result = 0
                else:
                    giro.angular.z = 0.5
                    self.result = 0
        else: # If the ball is not found, rotate the robot faster
            giro.angular.z = 1
            self.result = 0
        pub.publish(giro)

    def execute(self, userdata):
        self.orientate()
        if self.result == 0: # If the ball is not found or not centered
            return 'notcentered'
        else: # If the ball is centered
            return 'centered'


# Define state Positions
class Positions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['result', 'notresult'])

        self.position = False
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.data_scan)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.data_odom)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.vector_scan = LaserScan().ranges
        self.vector_odom = Odometry().pose.pose
        global x # The ball position
        
        # Set the goal point depending on the ball position
        if x > 0:
            self.goal_point_x = 3
        else:
            self.goal_point_x = -3

        self.goal_point_y = 0 # It is always 0

        self.result = False
        self.average = 0
        self.destino = []
        self.theta_robot = 0
        self.move = ClienteMoveBase() # Create a client to move the robot to a desired point
         
    def data_scan(self, msg2): # Extract laser data
        self.vector_scan = msg2.ranges
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

    
    def compute_position(self):
        # Robot's orientation
        q = [self.vector_odom.orientation.x, self.vector_odom.orientation.y, self.vector_odom.orientation.z, self.vector_odom.orientation.w] 
        self.theta_robot = tf.transformations.euler_from_quaternion(q)[2]

        # The x-axis of the robot points directly to the ball
        # Ball position relative to the robot
        pos_ball_robot = np.array([self.average, 0, 0-self.theta_robot])

        # Robot position
        pos_robot_world = np.array([self.vector_odom.position.x, self.vector_odom.position.y, self.theta_robot])

        # Z-axis rotation matrix
        mat_rot_z = np.array([[math.cos(self.theta_robot), -math.sin(self.theta_robot), 0],[math.sin(self.theta_robot), math.cos(self.theta_robot), 0],[0,0,1]])     

        # Position of the ball in relation to the world
        pos_ball_world = np.dot(mat_rot_z,pos_ball_robot) + pos_robot_world

        # Orientation of the ball with respect to the point representing the goal
        orientation_ball = math.atan2((self.goal_point_y - pos_ball_world[1]), (self.goal_point_x- pos_ball_world[0]))
        dist = 1.2 # Distance to keep between the robot and the ball
        # Position to be reached by the robot
        self.destino = [pos_ball_world[0] - dist*math.cos(orientation_ball), pos_ball_world[1] - dist*math.sin(orientation_ball), orientation_ball]

    def execute(self, userdata):
        if self.average != 0 and not self.position: # Positions only have to be computed once
            self.position = True
            self.compute_position()
        movement = Twist() # Create the movement
        # Move the robot to the desired point
        self.result = self.move.moveTo(self.destino[0], self.destino[1], self.destino[2])
        
        # Robot orientation
        q = [self.vector_odom.orientation.x, self.vector_odom.orientation.y, self.vector_odom.orientation.z, self.vector_odom.orientation.w] 
        theta_r = tf.transformations.euler_from_quaternion(q)[2]
        
        # MoveTo is not quite accurate, so the robot must be orientated again
        while abs(self.destino[2] - theta_r) > 0.05:
            # Recalculate robot orientation
            q = [self.vector_odom.orientation.x, self.vector_odom.orientation.y, self.vector_odom.orientation.z, self.vector_odom.orientation.w] 
            theta_r = tf.transformations.euler_from_quaternion(q)[2]
            if self.destino[2] - theta_r > 0:
                movement.angular.z = 0.5
                self.pub.publish(movement)
            else:
                movement.angular.z = -0.5
                self.pub.publish(movement)
                
        if self.result: # If the robot has arrived
            self.result = False
            return 'result'
        else: # If the movement could not be made
            self.move.client.cancel_goal() # Cancel the action
            return 'notresult'
        

# Define state Shoot
class Shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bumper'])
        self.bumped = False
        self.sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.is_bumper)

    def is_bumper(self, msg): # Extract the bumper sensor data
        if msg.state == 1:
            self.bumped = True
        else:
            self.bumped = False

    def move_forward(self):
        pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        go = Twist()
        c = 0
        while not self.bumped and c < 20: # Go forward until a collision is detected or 2 seconds have elapsed
            c = c + 1
            go.linear.x = 1
            pub.publish(go)
            rospy.sleep(0.1)

        go.linear.x = 0 # Stop the robot
        pub.publish(go)


    def execute(self, userdata):
        self.move_forward() # Move the robot and return the output to pass to the next node
        return 'bumper'
       

# Define state Repose
class Repose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gol', 'notgol'])
        self.sub = rospy.Subscriber('/goal', Bool, self.is_gol)
        self.gol = False

    def is_gol(self, msg): # Extract the goal data
        if msg == Bool(True):
            self.gol = True
        else:
            self.gol = False


    def execute(self, userdata):
        c = 0
        while not self.gol and c < 50: # Wait until a goal is detected or 5 seconds have elapsed
            c = c + 1
            rospy.sleep(0.1)

        if self.gol: # If it has been goal
            return 'gol'
        else: # If it has not been goal
            return 'notgol'

# Define state GoForward 
class GoForward(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['orientated', 'notorientated'])
        self.bridge = CvBridge()
        self.sub = rospy.Subscriber('/camera/rgb/image_raw', Image, self.callback)
        self.rango_medio = 10
    
    def callback(self, img):
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
            self.found = True # The ball is found
            self.x = m['m10']/m['m00']
        else:
            self.found = False

    def orientate(self): # Orient the robot to place ball in the center
        pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        giro = Twist() # Create the movement

        if self.found:
            if int(self.x) < self.width/2 + self.rango_medio and int(self.x) > self.width/2 - self.rango_medio:
                giro.angular.z = 0 # If the ball is in the center, the robot does not have to move 
                self.result = 1
            else: # If the ball is in the center, the robot does not have to move 
                if self.x > self.width/2:
                    giro.angular.z = -0.5
                    self.result = 0
                else:
                    giro.angular.z = 0.5
                    self.result = 0
        else: # If the ball is not found, rotate the robot faster
            giro.angular.z = 1
            self.result = 0
        pub.publish(giro)

    def execute(self, userdata):
        self.orientate()
        if self.result == 1: # If the robot is orientated
            return 'orientated'
        else: # If the robot is not orientated
            return 'notorientated'


# Main function
def main():
    rospy.init_node('smach_example_state_machine')
    start_ball = SpawnBall()
    start_ball.spawn_ball()
    
    global x
    x = start_ball.x # Spawn the ball
    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['GOL', 'NO GOL'])
    
    # Open the container
    with sm:
        # Add states to the container

        smach.StateMachine.add('BALL', Ball(), 
                               transitions={'centered':'POSITIONS', 
                                            'notcentered':'BALL'})

        
        smach.StateMachine.add('POSITIONS', Positions(), 
                               transitions={'result':'SHOOT', 
                                            'notresult':'GOFORWARD'})

        smach.StateMachine.add('SHOOT', Shoot(),
                                transitions={'bumper':'REPOSE'})

        smach.StateMachine.add('REPOSE', Repose(),
                                transitions={'gol':'GOL',
                                             'notgol':'NO GOL'})                                             
        
        smach.StateMachine.add('GOFORWARD', GoForward(),
                                transitions={'orientated':'SHOOT',
                                             'notorientated':'GOFORWARD'})  

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
