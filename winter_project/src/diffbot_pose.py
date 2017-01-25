#!/usr/bin/env python
import rospy
from std_msgs.msg import String
from sensor_msgs.msg import Pose2D
from geometry_msgs.msg import Twist, Vector3
from math import pi, atan, sin, cos

#service call?
#import TeleportAbsolute from somewhere else
#hold off on this for now

#maybe use JointState rather than Twist?, use Twist for now

#set up message fields/topics

def integrated_pose(Twist):
    '''
    -using set time step, integrate the kinematics of the differential
    drive robot to derive pose estimate (introduce noise later)
    -method used: Euler integration
    -converts velocities into pose estimates
    '''
    pose_new = Pose2D()

    pose_new.x = pose_x + (((Twist.linear.x)**2+(Twist.linear.y)**2)**(0.5)*cos(Twist.angular.z))*dt
    pose_new.y = pose_y + (((Twist.linear.x)**2+(Twist.linear.y)**2)**(0.5)*sin(Twist.angular.z))*dt
    pose_new_theta = pose_theta + (Twist.angular.z)*dt

    pub.publish(pose)

    pose_x = pose_new.x
    pose_y = pose_new.y
    pose_theta = pose_new_theta

    return

if __name__=='__main__':
    #initializations
    pose_x = 0
    pose_y = 0
    pose_theta = pi/2

    rospy.init_node('diffbot_pose')
    pub = rospy.Publisher('camera_pose', Pose2D, queue_size=10)
    rospy.Subscriber('pose_update', Twist, integrated_pose)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
