#!/usr/bin/env python

###############
# ROS IMPORTS #
###############
import rospy
import tf
import tf.transformations as tr
from geometry_msgs.msg import Twist, Pose2D
from winter_project.msg import multi_poses, multi_vels

###################
# NON ROS IMPORTS #
###################
import numpy as np
from math import pi, atan, sin, cos, sqrt

####################
# GLOBAL CONSTANTS #
####################
PARENT_FRAME = "odom"
CHILD_FRAME = "robot_1"
INTEGRATION_FREQ = 250 # Hz
# INITIAL_POSES = np.array([[1.,0.,0.]])
N = 30

# circle arrangement for 10 robots
# INITIAL_POSES = np.array([[3., 0., 0.]
#     ,[3.*sqrt(2.)/2.,3.*sqrt(2.)/2.,0.],
#     [0., 3., 0.],
#     [-3.*sqrt(2.)/2.,3.*sqrt(2.)/2.,0.],
#     [-3., 0., 0.],
#     [3.*cos(7.*pi/6.), 3*sin(7.*pi/6.), 0.],
#     [3.*cos(4.*pi/3.), 3*sin(4.*pi/3.), 0.],
#     [0., -3., 0.],
#     [3.*cos(5.*pi/3), 3.*sin(5.*pi/3.), 0.],
#     [3.*cos(11.*pi/6.), 3.*sin(11.*pi/6.), 0.]])


# star arrangement for 10 robots
# INITIAL_POSES = np.array([[-1., 0., 0.],[1., 0., 0.],
#     [0.5*sqrt(2.)/2.,0.5*sqrt(2.)/2.,0.],
#     [0., 1., 0.],
#     [-0.5*sqrt(2.)/2.,0.5*sqrt(2.)/2.,0.],
#     [1.*cos(4.*pi/3.), 1*sin(4.*pi/3.), 0.],
#     [0.5*cos(7.*pi/6.), 0.5*sin(7.*pi/6.), 0.],
#     [0., -0.5, 0.],
#     [1.*cos(5.*pi/3), 1.*sin(5.*pi/3.), 0.],
#     [0.5*cos(11.*pi/6.), 0.5*sin(11.*pi/6.), 0.]])

# cross arrangement for 10 robots
# INITIAL_POSES = np.array([[0., 1., 0.],[0., 0.5, 0.],[-1., 0., 0.],[-0.5, 0., 0.],[0., 0., 0.],[0.5, 0., 0.],[1., 0., 0.],[0., -0.5, 0.],[0., -1., 0.],[0., -2., 0.]])

# N arrangement
# INITIAL_POSES = np.array([[-2.5,-2.,0.],[-2.5,-1.5,0.],[-2.5,-1.,0.],[-2.5,-0.5,0.],[-2.5,0.,0.],[-2.5,0.5,0.],[-2.5,1.0,0.],[-2.5,1.5,0.],[-2.5,2.0,0.],[-2.5,2.5,0.],[-2.,2.0,0.],[-1.5,1.5,0.],[-1.,1.0,0.],[-0.5,0.5,0.],[0.,0.,0.],[0.5,-0.5,0.],[1.,-1.,0.],[1.5,-1.5,0.],[2.0,-2.0,0.],[2.5,-2.5,0.],[2.5,-2.0,0.],[2.5,-1.5,0.],[2.5,-1.0,0.],[2.5,-0.5,0.],[2.5,0.,0.],[2.5,0.5,0.],[2.5,1.0,0.],[2.5,1.5,0.],[2.5,2.0,0.],[2.5,2.5,0.]])

# U arrangement
# INITIAL_POSES = np.array([[-2.5,2.5,0.],[-2.5,2.,0.],[-2.5,1.5,0.],[-2.5,1.0,0.],[-2.5,0.5,0.],[-2.5,0.,0.],[-2.5,-0.5,0.],[-2.5,-1.,0.],[-2.5,-1.5,0.],[-2.25,-2.0,0.],[-2.0,-2.5,0.],[-1.5,-3.0,0.],[-1.0,-3.25,0.],[-0.5,-3.5,0.],[0.,-3.5,0.],[0.5,-3.5,0.],[1.0,-3.25,0.],[1.5,-3.0,0.],[2.0,-2.5,0.],[2.25,-2.,0.],[2.5,-1.5,0.],[2.5,-1.0,0.],[2.5,-0.5,0.],[2.5,0.,0.],[2.5,0.5,0.],[2.5,1.0,0.],[2.5,1.5,0.],[2.5,2.0,0.],[2.5,2.5,0.],[2.5,3.0,0.]])

# loop arrangement
INITIAL_POSES = np.array([[1., 0., 0.]
    ,[1.*sqrt(2.)/2.,1.*sqrt(2.)/2.,0.],
    [0., 1., 0.],
    [-1.*sqrt(2.)/2.,1.*sqrt(2.)/2.,0.],
    [-1., 0., 0.],
    [1.*cos(7.*pi/6.), 1*sin(7.*pi/6.), 0.],
    [1.*cos(4.*pi/3.), 1*sin(4.*pi/3.), 0.],
    [0., -1., 0.],
    [1.*cos(5.*pi/3), 1.*sin(5.*pi/3.), 0.],
    [1.*cos(11.*pi/6.), 1.*sin(11.*pi/6.), 0.],
[1.-2., 0., 0.]
    ,[1.*sqrt(2.)/2.-2.,1.*sqrt(2.)/2.,0.],
    [0.-2., 1., 0.],
    [-1.*sqrt(2.)/2.-2.,1.*sqrt(2.)/2.,0.],
    [-1.-2., 0., 0.],
    [1.*cos(7.*pi/6.)-2., 1*sin(7.*pi/6.), 0.],
    [1.*cos(4.*pi/3.)-2., 1*sin(4.*pi/3.), 0.],
    [0.-2., -1., 0.],
    [1.*cos(5.*pi/3)-2., 1.*sin(5.*pi/3.), 0.],
    [1.*cos(11.*pi/6.)-2., 1.*sin(11.*pi/6.), 0.],
[1.+2., 0., 0.]
    ,[1.*sqrt(2.)/2.+2.,1.*sqrt(2.)/2.,0.],
    [0.+2., 1., 0.],
    [-1.*sqrt(2.)/2.+2.,1.*sqrt(2.)/2.,0.],
    [-1.+2., 0., 0.],
    [1.*cos(7.*pi/6.)+2., 1*sin(7.*pi/6.), 0.],
    [1.*cos(4.*pi/3.)+2., 1*sin(4.*pi/3.), 0.],
    [0.+2., -1., 0.],
    [1.*cos(5.*pi/3)+2., 1.*sin(5.*pi/3.), 0.],
    [1.*cos(11.*pi/6.)+2., 1.*sin(11.*pi/6.), 0.]])


# smile arrangement
# INITIAL_POSES = np.array([[-2.5, 2.5, 0.],[-2.5, 2.25, 0.],[-2.5, 2., 0.],[-2.5, 1.75, 0.],[-2.5, 1.5, 0.],[-2.5, 1.25, 0.],[-2.5, 1.0, 0.],[-2.5, 0.75, 0.],[-2.5,-1.5,0.],[-2.25,-2.0,0.],[-2.0,-2.5,0.],[-1.5,-3.0,0.],[-1.0,-3.25,0.],[-0.5,-3.5,0.],[0.,-3.5,0.],[0.5,-3.5,0.],[1.0,-3.25,0.],[1.5,-3.0,0.],[2.0,-2.5,0.],[2.25,-2.,0.],[2.5,-1.5,0.],[2.5, 2.5, 0.],[2.5, 2.25, 0.],[2.5, 2., 0.],[2.5, 1.75, 0.],[2.5, 1.5, 0.],[2.5, 1.25, 0.],[2.5, 1.0, 0.],[2.5, 0.75, 0.],[0.,0.,0.]])

# CREATE MAIN CLASS:
class DiffDriveIntegrator( object ):
    def __init__(self):
        #rospy.loginfo("Creating DiffDriveIntegrator class!")

        # read all parameters:
        self.parent = rospy.get_param("~parent", PARENT_FRAME)
        self.child = rospy.get_param("~child", CHILD_FRAME)
        self.freq = rospy.get_param("~freq", INTEGRATION_FREQ)

        # create random variables that we are going to need:
        self.current_vel = np.zeros((N,2))
        self.current_pose = INITIAL_POSES
        self.n_poses = multi_poses()

        self.init_flag = False

        # self.size_flag = True
        # create all subscribers, timers, publishers, listeners, broadcasters,
        # etc.
        self.br = tf.TransformBroadcaster()
        self.pos_pub = rospy.Publisher("pose_est", multi_poses, queue_size=10)
        self.cmd_sub = rospy.Subscriber("cmd_vel", multi_vels, self.twist_callback)
        self.int_timer = rospy.Timer(rospy.Duration(1/float(self.freq)), self.integrate_callback)
        return

    def integrate_callback(self, tdata):
        self.n_poses = multi_poses()
        #function
        # print self.current_vel
        for i in range(N):

            #casting iteration counter as integer
            # i = i[0]

            v = self.current_vel[i][0]
            w = self.current_vel[i][1]

            # if not self.init_flag:
            #     self.current_pose[i] = INITIAL_POSES[i]
            #     self.init_flag = True

            # print "INTEGRATOR NODE POSE", self.current_pose[i]
            # print type(self.current_pose[0])
            # print "self.current_pose[2]", self.current_pose[0][2]

            self.current_pose[i] += (1/float(self.freq))*np.array([
                v*np.cos(self.current_pose[i][2]),
                v*np.sin(self.current_pose[i][2]),
                w])

            # let's send that pose data out as a tf message:
            pos, quat = self.pose_arr_to_tf(self.current_pose[i])
            self.child = "robot"+"_"+str(i)
            self.br.sendTransform(pos, quat, rospy.Time.now(), self.child, self.parent)

            #publish estimated pose to diffdrive_velocity controller via pose_est topic

            self.n_poses.poses.append(Pose2D(self.current_pose[i][0],self.current_pose[i][1],self.current_pose[i][2]))
            # self.init_flag = False
        self.pos_pub.publish(self.n_poses)

        return

    def twist_callback(self, command):
        self.current_vel = self.twist_to_vel_arr(command)
        return

    def pose_arr_to_tf(self, pose):
        pos = [pose[0], pose[1], 0]
        quat = tr.quaternion_from_euler(0, 0, pose[2], 'sxyz')
        return pos, quat

    def twist_to_vel_arr(self, multi):
        vel = np.zeros((N,2))
        for i in range(N):
            vel[i] = np.array([multi.twists[i].linear.x, multi.twists[i].angular.z])
        return vel


def main():
    rospy.init_node('formation_diff_drive_integrator', log_level=rospy.INFO)

    try:
        diffdrive = DiffDriveIntegrator()

    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__=='__main__':
    main()

