#!/usr/bin/env python

###############
# ROS IMPORTS #
###############
import rospy
from geometry_msgs.msg import Twist, Pose2D

###################
# NON ROS IMPORTS #
###################
import numpy as np
from math import pi, atan, sin, cos


class TrajectoryGeneration():
    def __init__(self):

        #initialize timer
        self.t0 = rospy.get_time()

        #initialize publisher
        self.pub = rospy.Publisher('pose_est', Pose2D, queue_size=10)
        self.rate = rospy.Rate(250)
        return

    def parametric(self):

        t = rospy.get_time() - self.t0

        x_t = 4.*cos(t)
        y_t = 4.*sin(t)

        #how to estimate local heading of each robot on trajectory?

        #instance of Pose2D, which will be published to pose_est topic that velocity controller subscribes to
        trajectory_coordinates = Pose2D()

        trajectory_coordinates.x = x_t
        trajectory_coordinates.y = y_t

        pub.publish(trajectory_coordinates)
        return

def main():
    rospy.init_node('diff_drive_trajectory_generation')

    try:
        trajectory_generator = TrajectoryGeneration()
    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__=='__main__':
    main()
