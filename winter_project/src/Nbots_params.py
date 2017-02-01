#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Pose2D, Point, Pose, Quaternion
from winter_project.msg import coords_msgs
from math import pi, atan, sin, cos

#initialize publisher and tf
rospy.init_node('diffbot_coords')
pub = rospy.Publisher('/robot_coords', coords_msgs, queue_size=10)
rate = rospy.Rate(10)

#robot1

#specify starting pose and goal pose
#start
pose_x = 0
pose_y = 0
pose_theta = 0

#goal
x_final = 0
y_final = 10
theta_final = pi/2

rob1_coords = coords_msgs()

rob1_coords.pose_x = pose_x
rob1_coords.pose_y = pose_y
rob1_coords.pose_theta = pose_theta

rob1_coords.x_final = x_final
rob1_coords.y_final = y_final
rob1_coords.theta_final = theta_final

pub.publish(rob1_coords)

"""
#robot2

#specify starting pose and goal pose
#start
pose_x = 5
pose_y = 0
pose_theta = 0

#goal
x_final = 5
y_final = 10
theta_final = pi/2
"""
