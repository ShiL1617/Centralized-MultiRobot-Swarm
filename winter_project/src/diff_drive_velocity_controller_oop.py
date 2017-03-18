#!/usr/bin/env python

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose2D

#NON ROS IMPORTS
from math import pi, atan, sin, cos, tan, atan2, floor, ceil
import numpy as np

#GLOBAL CONSTANTS
CONTROLLER_NUM = 0

#create class
class DiffDriveVelocityController( object ):
    def __init__(self, GOAL_POSE):
        self.K1P = 1.0
        self.K2P = 1.0

        #read parameters, try to have goal pose as a parameter that can be set in launch file (easier for multiple robots)
        #self.goal_pose = rospy.get_param("~goalpose",GOAL_POSE)
        self.controller_num = rospy.get_param("~controller_num", CONTROLLER_NUM)
        #needed random variables
        self.goal_pose_list = GOAL_POSE
        self.goal_pose = None
        # self.goal_pose = GOAL_POSE[self.controller_num]
        self.val = False
        self.temp = 0
        self.x_init = 0
        self.y_init = 0

        self.global_rotation_error = 99
        self.global_rotation_direction = 99

        self.linear_error = 99

        self.body_rotation_error = 99
        self.body_rotation_direction = 99


        #initialize publishers and subscribers
        self.pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
        self.rate = rospy.Rate(250)
        self.sub = rospy.Subscriber('pose_est', Pose2D, self.pose_callback)


        return

    def pose_callback(self,pose2D):

        vel = Twist()

        linear_error = ((self.goal_pose[0]-pose2D.x)**2+(self.goal_pose[1]-pose2D.y)**2)**(0.5)

        if not self.val:
            self.temp = linear_error
            self.x_init = pose2D.x
            self.y_init = pose2D.y
            self.val = True
            #print self.temp

        #applying mod 2*pi to unwrap the current pose estimate orientation
        global_init_theta = ((pose2D.theta))%(2.*pi)
        global_goal_theta = atan2(self.goal_pose[1]-pose2D.y,self.goal_pose[0]-pose2D.x)%(2.*pi)

        #dy = abs(pose2D.y - self.goal_pose[1])
        dy = abs(self.y_init - self.goal_pose[1])
        #if x coordinate of goal pose is larger than initial x coordinate
        if self.x_init < self.goal_pose[0]:
            a_cornercase, r_cornercase = self.turn(global_init_theta, 2.*pi)
        #if x of goal pose is less than " "
        elif self.x_init >= self.goal_pose[0]:
            a_cornercase, r_cornercase = self.turn(global_init_theta, pi)

        elif self.x_init == self.goal_pose[0]:
            #comparing y coords
            if self.y_init > self.goal_pose[1]:
                a_cornercase, r_cornercase = self.turn(global_init_theta, pi/2.)
            elif self.y_init < self.goal_pose[1]:
                a_cornercase, r_cornercase = self.turn(global_init_theta, 3.*pi/2.)
            else:
                a_cornercase = 0

        a_first, r_first = self.turn(global_init_theta, global_goal_theta)
        a_final, r_final = self.turn(pose2D.theta, self.goal_pose[2])

        if dy < 0.04:

            if abs(a_cornercase) > (pi/1000.) and self.temp == linear_error:
                vel.linear.x = 0.
                vel.angular.z = self.K2P*a_cornercase*r_cornercase
                #print "global_init_theta =", global_init_theta
                #print "hello"
            elif abs(linear_error) > 0.01:
                vel.linear.x = self.K1P*linear_error
                vel.angular.z = 0.
                #print "did i get here?"
            elif abs(a_final) > pi/100.:
                vel.linear.x = 0.
                vel.angular.z = self.K2P*a_final*r_final

            else:
                vel.linear.x = 0.
                vel.angular.z = 0.
                print "ended"

        else:
            if abs(a_first) > (pi/100.) and abs(linear_error)>0.01:
                vel.linear.x = 0.
                vel.angular.z = self.K2P*a_first*r_first

            elif abs(linear_error) > 0.01:
                vel.linear.x = self.K1P*linear_error
                vel.angular.z = 0.

            elif abs(a_final) > pi/100.:
                vel.linear.x = 0.
                vel.angular.z = self.K2P*a_final*r_final

            else:
                vel.linear.x = 0.
                vel.angular.z = 0.
        """
        #testing multiple robots driving in concentric circles
        if self.controller_num == 0:
            vel.linear.x = 1.
            vel.angular.z = 1.
        elif self.controller_num == 1:
            vel.linear.x = 1.
            vel.angular.z = 1.
        elif self.controller_num == 2:
            vel.linear.x = 1.
            vel.angular.z = 1.
        """
        self.pub.publish(vel)
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

    GOAL_POSE = np.array([[1.,1., 0.],[2.,2.,0.],[3.,3.,0.]])

    try:
        controller = DiffDriveVelocityController(GOAL_POSE)

    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__=='__main__':
    main()
