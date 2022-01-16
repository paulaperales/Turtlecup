#!/usr/bin/env python

import rospy, tf
from gazebo_msgs.srv import DeleteModel, SpawnModel
from geometry_msgs.msg import *
import numpy as np

if __name__ == '__main__':
    print("Waiting for gazebo services...")
    rospy.init_node("spawn_products_in_bins")
    rospy.wait_for_service("gazebo/delete_model")
    rospy.wait_for_service("gazebo/spawn_urdf_model")
    print("Got it.")
    delete_model = rospy.ServiceProxy("gazebo/delete_model", DeleteModel)
    spawn_model = rospy.ServiceProxy("gazebo/spawn_urdf_model", SpawnModel)
    delete_model("ball_description")

    with open("/home/paula/tb2_ws/src/turtlebot2/turtlebot_simulator/turtlebot_gazebo/maps/ball.urdf.xacro", "r") as f:
        product_xml = f.read()  

    x_aleat = np.random.choice([1.19211, -1.19211], size=1, p=[0.5,0.5])
    s = np.random.uniform(-1, 1, 100)
    y_aleat = np.random.choice(s, size=1)
    item_pose = Pose(position=Point(x=x_aleat, y=y_aleat, z=0), orientation=Quaternion(0,0,0,0))
    
    spawn_model("ball_description", product_xml, "", item_pose, "world")