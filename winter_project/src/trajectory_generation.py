#!/usr/bin/env python

###############
# ROS IMPORTS #
###############
import rospy
from geometry_msgs.msg import Twist, Pose2D
from winter_project.msg import multi_poses, multi_vels

###################
# NON ROS IMPORTS #
###################
import numpy as np
from math import pi, atan2, sin, cos

N = 10

class TrajectoryGeneration():
    def __init__(self):

        #initialize timer
        self.t0 = rospy.get_time()

        #initialize multi_poses message to be published
        self.traj = multi_poses()

        #initialize publisher
        self.pub = rospy.Publisher('pose_des', multi_poses, queue_size=10)
        self.rate = rospy.Rate(50)

        return

    def parametric(self):
        self.traj = multi_poses()

        t = rospy.get_time() - self.t0

        for i in range(N):

            phase_var = i*0.1

            x_t = 4.*cos(t-i)
            y_t = 4.*sin(t-i)

            dx_dt = -4*sin(t)
            dy_dt = 4*cos(t)

            theta_t = atan2(dy_dt,dx_dt)

            self.traj.poses.append(Pose2D(x_t, y_t, theta_t))

        pub.publish(self.traj)
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
