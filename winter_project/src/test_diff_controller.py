#!/usr/bin/env python

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose2D
from winter_project.msg import multi_poses, multi_vels
#NON ROS IMPORTS
from math import pi, atan, sin, cos, tan, atan2, floor, ceil
import numpy as np

#GLOBAL CONSTANTS
#CONTROLLER_NUM = 0
GOAL_POSES = np.array([[1.,1.,0.],[2., 2., 0.]])

#create class
class DiffDriveVelocityController( object ):
    def __init__(self, robot_number):
        self.K1P = 1.0
        self.K2P = 1.0

        #read parameters, try to have goal pose as a parameter that can be set in launch file (easier for multiple robots)
        #self.goal_pose = rospy.get_param("~goalpose",GOAL_POSE)
        #self.controller_num = rospy.get_param("~controller_num", "robot"+"_"+str(robot_number))
        #needed random variables
        self.goal_pose = GOAL_POSES[robot_number]
        #print "goal_pose"+str(robot_number)+"=", self.goal_pose
        self.robot_number = robot_number
        # print "self.robot_number=", self.robot_number
        self.robot_name = "robot"+"_"+str(robot_number)

        self.multi_vels = multi_vels()
        self.multi_vels.robot_name = self.robot_name
        self.multi_vels.twists.append(Twist(Vector3(0.,0.,0.),Vector3(0.,0.,0.)))

        #set flags
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
        self.pub = rospy.Publisher('cmd_vel', multi_vels, queue_size=10)
        self.rate = rospy.Rate(50)
        self.sub = rospy.Subscriber('pose_est', multi_poses, self.pose_callback)


        return

    def pose_callback(self,poses):

        vel = self.multi_vels
        vel.robot_name = poses.robot_name
        robot_number = int(vel.robot_name[6])

        # print "robot_name is:", vel.robot_name
        # print "robot_number is:", robot_number

        V = Twist()

        pose2Dx = poses.pose_ests[robot_number].x
        pose2Dy = poses.pose_ests[robot_number].y
        pose2Dtheta = poses.pose_ests[robot_number].theta

        # print "goal pose is:", self.goal_pose ????
        # print "robot number:", robot_number
        # print "pose2Dx = ", pose2Dx
        # print "pose2Dy = ", pose2Dy
        # print "pose2Dtheta = ", pose2Dtheta

        # print "goal pose is:", self.goal_pose
        linear_error = ((self.goal_pose[0]-pose2Dx)**2+(self.goal_pose[1]-pose2Dy)**2)**(0.5)

        if not self.val:
            self.temp = linear_error
            self.x_init = pose2Dx
            self.y_init = pose2Dy
            self.val = True
            #print self.temp

        #applying mod 2*pi to unwrap the current pose estimate orientation
        global_init_theta = ((pose2Dtheta))%(2.*pi)
        global_goal_theta = atan2(self.goal_pose[1]-pose2Dy,self.goal_pose[0]-pose2Dx)%(2.*pi)

        dy = abs(self.y_init - self.goal_pose[1])
        #print "dy=", dy
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
        a_final, r_final = self.turn(pose2Dtheta, self.goal_pose[2])

        if dy < 0.04:

            if abs(a_cornercase) > (pi/1000.) and self.temp == linear_error:
                V.linear.x = 0.
                V.angular.z = self.K2P*a_cornercase*r_cornercase
                #print "global_init_theta =", global_init_theta
                #print "first_turn_corner_case"
            elif abs(linear_error) > 0.01:
                V.linear.x = self.K1P*linear_error
                V.angular.z = 0.
                #print "drive_forward_corner_case"
            elif abs(a_final) > pi/100.:
                V.linear.x = 0.
                V.angular.z = self.K2P*a_final*r_final
                #print "final_turn_corner_case"
            else:
                V.linear.x = 0.
                V.angular.z = 0.
                #print "ended_corner_case"

        else:
            if abs(a_first) > (pi/50.) and abs(linear_error)>0.01:
                V.linear.x = 0.
                V.angular.z = self.K2P*a_first*r_first
                #print "first turn"
            elif abs(linear_error) > 0.01:
                V.linear.x = self.K1P*linear_error
                V.angular.z = 0.
                #print "drive forward"
            elif abs(a_final) > pi/50.:
                V.linear.x = 0.
                V.angular.z = self.K2P*a_final*r_final
                #print "final turn"
            else:
                V.linear.x = 0.
                V.angular.z = 0.
                #print "ended"

        vel.twists[robot_number].linear.x = V.linear.x
        vel.twists[robot_number].angular.z = V.angular.z

        self.pub.publish(vel)
        self.rate.sleep()
        return vel

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
    rospy.init_node('test_diff_drive_velocity_controller')

    N = len(GOAL_POSES)

    try:
        for i in range(N):
            DiffDriveVelocityController(i)

    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__=='__main__':
    main()
