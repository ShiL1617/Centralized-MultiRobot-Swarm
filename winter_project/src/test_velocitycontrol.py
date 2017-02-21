#!/usr/bin/env python

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose2D

#NON ROS IMPORTS
from math import pi, atan, sin, cos, atan2
import numpy as np

"""
    """
    def first_turn(self, pose2D):
        t1 = (pose2D.theta)%(2.*pi)
        t2 = atan2(self.goal_pose[1]-pose2D.y,self.goal_pose[0]-pose2D.x)%(2.*pi)

        if abs(t1 - t2) >= pi:
            self.global_rotation_error = 2*pi-abs(t1 - t2)
        else:
            self.global_rotation_error = abs(t1 - t2)

        if t1 > t2:
            self.global_rotation_direction = -1
        else:
            self.global_rotation_direction = 1

    def final_turn(self, pose2D):
        t1 = pose2D.theta
        t2 = self.goal_pose[2]

        if abs(t1 - t2) >= pi:
            self.global_rotation_error = 2*pi-abs(t1 - t2)
        else:
            self.global_rotation_error = abs(t1 - t2)

        if t1 > t2:
            self.global_rotation_direction = -1
        else:
            self.global_rotation_direction = 1
    """
"""

class DiffDriveVelocityController( object ):
    def __init__(self, GOAL_POSE):
        self.K1P = 3.
        self.K2P = 3.

        #read parameters, try to have goal pose as a parameter that can be set in launch file (easier for multiple robots)
        #self.goal_pose = rospy.get_param("~goalpose",GOAL_POSE)

        #needed random variables
        self.goal_pose = GOAL_POSE

        #initialize publishers and subscribers
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(250)
        self.sub = rospy.Subscriber('pose_est', Pose2D, self.pose_callback)

        """
        self.global_rotation_error = 99
        self.global_rotation_direction = 99

        self.linear_error = 99

        self.body_rotation_error = 99
        self.body_rotation_direction = 99
        """
        return

    def pose_callback(self,pose2D):

        self.x = pose2D.x
        self.y = pose2D.y
        #self.theta = pose2D.theta

        vel = Twist()

        linear_error = ((self.goal_pose[0]-pose2D.x)**2+(self.goal_pose[1]-pose2D.y)**2)**(0.5)

        #applying mod 2*pi to unwrap the current pose estimate orientation
        global_init_theta = ((pose2D.theta))%(2*pi)

        global_goal_theta = atan2(self.goal_pose[1]-pose2D.y,self.goal_pose[0]-pose2D.x)%(2*pi)

        a_first, r_first = self.turn(global_init_theta, global_goal_theta)
        a_final, r_final = self.turn(pose2D.theta, self.goal_pose[2])

        #control law is P control
        if abs(a_first) > (pi/100.) and abs(linear_error)>0.01:
            vel.linear.x = 0.
            vel.angular.z = self.K2P*a_first*r_first
            # rospy.loginfo("First turn!")
            #print "global_orientation_direction:", self.rotation_direction_global
            print "global rotation error=", a_first

        elif abs(linear_error) > 0.01:
            vel.linear.x = self.K1P*linear_error
            vel.angular.z = 0.
            # rospy.loginfo("Driving forward!")
            print "linear_error:", linear_error

        elif abs(a_final) > pi/100.:
            vel.linear.x = 0.
            vel.angular.z = self.K2P*a_final*r_final
            # rospy.loginfo("Final Turn!")
            print "body_orientation_error:", a_final

        else:
            vel.linear.x = 0.
            vel.angular.z = 0.
            print "finished"
            rospy.loginfo("Finished!")


        """
        vel.linear.x = 1
        vel.angular.z = 1
        """

        self.pub.publish(vel)
        self.rate.sleep()
        return

    def turn(self, theta1, theta2):

        if abs(theta1 - theta2) >= pi:
            rotation_error = 2*pi-abs(theta1 - theta2)
        else:
            rotation_error= abs(theta1 - theta2)

        if theta1 > theta2:
            direction = -1
        else:
            direction = 1

        return rotation_error, direction

def main():
    rospy.init_node('diff_drive_velocity_controller')

    #can pass in a vector of goal poses into a loop, where I instantiate the DiffDriveVelocityController object (to create multiple velocity controllers for each robot)

    GOAL_POSE = np.array([3.,0., 3*pi/2.])

    controller = DiffDriveVelocityController(GOAL_POSE)

    rospy.spin()

if __name__=='__main__':
    main()
