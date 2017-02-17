#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Pose2D
from math import pi, atan, sin, cos

def coord2vel(pose2D):

    #gains K1, K2
    K1 = 5
    K2 = 1

    #goal position
    x_final = 1
    y_final = 1
    theta_final = pi/2

    vel = Twist()

    linear_error = ((x_final-pose2D.x)**2+(y_final-pose2D.y)**2)**(0.5)

    #account for angle wrapping cases, not done yet
    angular_error = abs(theta_final-(pi/2-(pose2D.theta%(2*pi))))

    vel.linear.x = (K1*linear_error*cos(angular_error))
    vel.linear.y = 0
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = K1*sin(angular_error)*cos(angular_error) + K2*angular_error

    pub.publish(vel)

    return

if __name__=='__main__':

    rospy.init_node('diffbot_velocity_controller')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('pose_est', Pose2D, coord2vel)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
