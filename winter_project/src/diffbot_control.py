#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Pose2D
from geometry_msgs.msg import Twist, Vector3
from math import pi, atan, sin, cos

def coord2vel(Pose2D):

    #gains K1, K2
    K1 = 20
    K2 = 20

    vel = Twist()

    linear_error = ((Pose2D.x-pose_x)**2+(Pose2D.y-pose_y)**2)**(0.5)
    angular_error = abs(theta_goal-Pose2D.theta)

    vel.linear.x = (K1*linear_error*cos(angular_error))*cos(Pose2D.theta)
    vel.linear.y = (K1*linear_error*cos(angular_error))*sin(Pose2D.theta)
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = K1*sin(angular_error)*cos(angular_error) + K2*angular_error

    pub.publish(vel)

    #updating/storing for next iteration
    pose_x = Pose2D.x
    pose_y = Pose2D.y
    pose_theta = Pose2D.theta

    return

if __name__=='__main__':

    rospy.init_node('diffbot_control')
    pub = rospy.Publisher('pose_update', Twist, queue_size=10)
    rospy.Subscriber('vel_update', Pose2D, coord2vel)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
