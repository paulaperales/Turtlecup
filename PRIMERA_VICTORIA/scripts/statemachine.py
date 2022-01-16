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
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib_msgs.msg import GoalStatus
from kobuki_msgs.msg import BumperEvent
from std_msgs.msg import Bool

class ClienteMoveBase:
    def __init__(self):
        #creamos un cliente ROS para la acción, necesitamos el nombre del nodo 
        #y la clase Python que implementan la acción
        #Para mover al robot, estos valores son "move_base" y MoveBaseAction
        self.client =  actionlib.SimpleActionClient('/move_base',MoveBaseAction)
        #esperamos hasta que el nodo 'move_base' esté activo`
        self.client.wait_for_server()

    def moveTo(self, x, y, orientation):
        #un MoveBaseGoal es un punto objetivo al que nos queremos mover
        goal = MoveBaseGoal()
        #sistema de referencia que estamos usando
        goal.target_pose.header.frame_id = "map"
        goal.target_pose.pose.position.x = x
        goal.target_pose.pose.position.y = y
        #La orientación es un quaternion. Tenemos que fijar alguno de sus componentes
        q = tf.transformations.quaternion_from_euler(0,0,orientation)
        goal.target_pose.pose.orientation.x = q[0]
        goal.target_pose.pose.orientation.y = q[1]
        goal.target_pose.pose.orientation.z = q[2]
        goal.target_pose.pose.orientation.w = q[3]
        #enviamos el goal 
        self.client.send_goal(goal)
        #vamos a comprobar cada cierto tiempo si se ha cumplido el goal
        #get_state obtiene el resultado de la acción 
        state = self.client.get_state()
        cont = 0
        #ACTIVE es que está en ejecución, PENDING que todavía no ha empezado
        while (state==GoalStatus.ACTIVE or state==GoalStatus.PENDING) and cont < 100:
            rospy.Rate(10)   #esto nos da la oportunidad de escuchar mensajes de ROS
            state = self.client.get_state()
            cont = cont + 1
            rospy.sleep(0.1)
        return self.client.get_result()

# define state Ball
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
            self.found = True
            self.x = m['m10']/m['m00']
        else:
            self.found = False
            

    def orientate(self):
        pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        giro = Twist()

        if self.found:
            if int(self.x) < self.width/2 + self.rango_medio and int(self.x) > self.width/2 - self.rango_medio:
                giro.angular.z = 0
                self.result = 1
            else:
                if self.x > self.width/2:
                    giro.angular.z = -0.5
                    self.result = 0
                else:
                    giro.angular.z = 0.5
                    self.result = 0
        else:
            giro.angular.z = 1
            self.result = 0
        pub.publish(giro)

    def execute(self, userdata):
        self.orientate()
        if self.result == 0:
            return 'notcentered'
        else:
            return 'centered'

# define state Positions
class Positions(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['result', 'notresult'])

        self.position = False
        self.sub_scan = rospy.Subscriber('/scan', LaserScan, self.data_scan)
        self.sub_odom = rospy.Subscriber('/odom', Odometry, self.data_odom)
        self.pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)

        self.vector_scan = LaserScan().ranges
        self.vector_odom = Odometry().pose.pose
        self.goal_point_x = 3
        self.goal_point_y = 0
        self.result = False
        self.average = 0
        self.destino = []
        self.theta_robot = 0
        self.move = ClienteMoveBase()



        
    def data_scan(self, msg2):
        self.vector_scan = msg2.ranges
        average = 0
        c = 0
        size = int(len(self.vector_scan)/2)
        for i in range(-5,5):
            if not np.isnan(self.vector_scan[size + i]):
                c = c + 1
                average = average + self.vector_scan[size + i]
        if c != 0:
            self.average = average/c
        else:
            self.average = self.vector_scan[size/2]

    
    def data_odom(self, msg):
        self.vector_odom = msg.pose.pose

    
    def compute_position(self):
        # Orientación robot
        q = [self.vector_odom.orientation.x, self.vector_odom.orientation.y, self.vector_odom.orientation.z, self.vector_odom.orientation.w] 
        self.theta_robot = tf.transformations.euler_from_quaternion(q)[2]

        # El eje x del robot apunta directamente a la pelota;
        # Posición pelota respecto al robot
        pos_ball_robot = np.array([self.average, 0, 0-self.theta_robot])
        print("POSICIÓN PELOTA RESPECTO AL ROBOT: ", pos_ball_robot)

        # Posición robot
        pos_robot_world = np.array([self.vector_odom.position.x, self.vector_odom.position.y, self.theta_robot])
        print("POSICIÓN ROBOT RESPECTO AL MUNDO: ", pos_robot_world)

        # Matriz rotación eje Z
        mat_rot_z = np.array([[math.cos(self.theta_robot), -math.sin(self.theta_robot), 0],[math.sin(self.theta_robot), math.cos(self.theta_robot), 0],[0,0,1]])     

        # Posición pelota respecto al mundo
        pos_ball_world = np.dot(mat_rot_z,pos_ball_robot) + pos_robot_world
        print("POSICIÓN PELOTA RESPECTO AL MUNDO: ", pos_ball_world)

        # Orientación pelota respecto al punto que representa la portería
        orientation_ball = math.atan2((self.goal_point_y - pos_ball_world[1]), (self.goal_point_x- pos_ball_world[0]))
        print(math.degrees(orientation_ball))
        dist = 1.2 # Distancia que mantener entre el robot y la pelota
        # Posición a la que tiene que llegar el robot
        self.destino = [pos_ball_world[0] - dist*math.cos(orientation_ball), pos_ball_world[1] - dist*math.sin(orientation_ball), orientation_ball]
        print("POSICIÓN A LA QUE TIENE QUE LLEGAR EL ROBOT: ", self.destino)
        # if self.destino[2] < 0:
        #     self.destino[2] = self.destino[2] + 2*math.pi
        # print(self.destino[0], self.destino[1], math.degrees(self.destino[2]))

      

    def execute(self, userdata):
        if self.average != 0 and not self.position:
            self.position = True
            self.compute_position()
        movement = Twist()
        print("DESTINO", self.destino)
        self.result = self.move.moveTo(self.destino[0], self.destino[1], self.destino[2])
        q = [self.vector_odom.orientation.x, self.vector_odom.orientation.y, self.vector_odom.orientation.z, self.vector_odom.orientation.w] 
        theta_r = tf.transformations.euler_from_quaternion(q)[2]
        print("ANGULO: ", math.degrees(self.destino[2]), "ANGULO ROBOT:", math.degrees(theta_r))
        while abs(self.destino[2] - theta_r) > 0.05:
            q = [self.vector_odom.orientation.x, self.vector_odom.orientation.y, self.vector_odom.orientation.z, self.vector_odom.orientation.w] 
            theta_r = tf.transformations.euler_from_quaternion(q)[2]
            if self.destino[2] - theta_r > 0:
                movement.angular.z = 0.5
                self.pub.publish(movement)
            else:
                movement.angular.z = -0.5
                self.pub.publish(movement)
        if self.result:
            self.result = False
            return 'result'
        else:
            return 'notresult'
        

# define state Shoot
class Shoot(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['bumper', 'notbumper'])
        self.bumped = False
        self.sub = rospy.Subscriber('/mobile_base/events/bumper', BumperEvent, self.is_bumper)

    def is_bumper(self, msg):
        if msg.state == 1:
            self.bumped = True
        else:
            self.bumped = False

    def move_forward(self):
        pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        go = Twist()
        if not self.bumped:
            go.linear.x = 1
            pub.publish(go)
        else:
            go.linear.x = 0
            pub.publish(go)


    def execute(self, userdata):
        self.move_forward()
        if self.bumped:
            return 'bumper'
        else:
            return 'notbumper'
       

# define state Repose
class Repose(smach.State):
    def __init__(self):
        smach.State.__init__(self, outcomes=['gol', 'notgol'])
        self.sub = rospy.Subscriber('/porteria_azul', Bool, self.is_gol)
        self.gol = False

    def is_gol(self, msg):
        if msg == Bool(True):
            self.gol = True
        else:
            self.gol = False


    def execute(self, userdata):

        c = 0

        while not self.gol and c < 50:
            c = c + 1
            rospy.sleep(0.1)


        if self.gol:
            return 'gol'
        else:
            return 'notgol'

# define state GoForward 
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
            self.found = True
            self.x = m['m10']/m['m00']
        else:
            self.found = False

    def orientate(self):
        pub = rospy.Publisher('/mobile_base/commands/velocity', Twist, queue_size=1)
        giro = Twist()

        if self.found:
            if int(self.x) < self.width/2 + self.rango_medio and int(self.x) > self.width/2 - self.rango_medio:
                giro.angular.z = 0
                self.result = 1
            else:
                if self.x > self.width/2:
                    giro.angular.z = -0.5
                    self.result = 0
                else:
                    giro.angular.z = 0.5
                    self.result = 0
        else:
            giro.angular.z = 1
            self.result = 0
        pub.publish(giro)

    def execute(self, userdata):
        self.orientate()
        if self.result == 1:
            return 'orientated'
        else:
            return 'notorientated'


# main
def main():
    
    rospy.init_node('smach_example_state_machine')

    # Create a SMACH state machine
    sm = smach.StateMachine(outcomes=['GOL'])

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
                                transitions={'bumper':'REPOSE',
                                             'notbumper':'SHOOT'})
        smach.StateMachine.add('REPOSE', Repose(),
                                transitions={'gol':'GOL',
                                             'notgol':'BALL'})                                             
        
        smach.StateMachine.add('GOFORWARD', GoForward(),
                                transitions={'orientated':'SHOOT',
                                             'notorientated':'GOFORWARD'})  

    # Execute SMACH plan
    outcome = sm.execute()


if __name__ == '__main__':
    main()
