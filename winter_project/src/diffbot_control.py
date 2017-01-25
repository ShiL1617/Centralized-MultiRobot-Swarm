#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Pose2D
from math import pi, atan, sin, cos

def coord2vel(Pose2D):

    #gains K1, K2
    K1 = 20
    K2 = 20

    #goal position
    x_final = 10
    y_final = 15
    theta_final = pi/4

    vel = Twist()

    linear_error = ((x_final-Pose2D.x)**2+(y_final-Pose2D.y)**2)**(0.5)
    angular_error = abs(theta_final-Pose2D.theta)

    vel.linear.x = (K1*linear_error*cos(angular_error))*cos(Pose2D.theta)
    vel.linear.y = (K1*linear_error*cos(angular_error))*sin(Pose2D.theta)
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = K1*sin(angular_error)*cos(angular_error) + K2*angular_error

    pub.publish(vel)

    #updating/storing for next iteration
    """
    pose_x = Pose2D.x
    pose_y = Pose2D.y
    pose_theta = Pose2D.theta
    """

    return

if __name__=='__main__':

    rospy.init_node('diffbot_control')
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/vel_update', Pose2D, coord2vel)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
