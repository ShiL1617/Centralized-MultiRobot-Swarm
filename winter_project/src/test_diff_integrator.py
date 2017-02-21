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
from math import pi, atan, sin, cos

####################
# GLOBAL CONSTANTS #
####################
PARENT_FRAME = "odom"
CHILD_FRAME = "robot_1"
INTEGRATION_FREQ = 50 # Hz
INITIAL_POSES = np.array([[0.,0.,0.],[1., 1., 0.]])
# INTEGRATOR_NUM = 0

# CREATE MAIN CLASS:
class DiffDriveIntegrator( object ):
    def __init__(self, robot_number):
        #rospy.loginfo("Creating DiffDriveIntegrator class!")

        # read all parameters:
        self.parent = rospy.get_param("~parent", PARENT_FRAME)
        self.child = rospy.get_param("~child", "robot"+"_"+str(robot_number))
        self.freq = rospy.get_param("~freq", INTEGRATION_FREQ)
        self.robot_name = "robot"+"_"+str(robot_number)

        # print "robot name:", self.robot_name

        self.p_est = multi_poses()
        self.p_est.robot_name = self.robot_name

        # print "initial pose =", [INITIAL_POSES[robot_number][0],INITIAL_POSES[robot_number][1],INITIAL_POSES[robot_number][2]]

        self.p_est.pose_ests.append(Pose2D(INITIAL_POSES[robot_number][0],INITIAL_POSES[robot_number][1],INITIAL_POSES[robot_number][2]))


        # create random variables that we are going to need:
        self.robot_number= robot_number
        self.current_pose = INITIAL_POSES[robot_number]
        # print "initial_pose"+str(robot_number)+"=", INITIAL_POSES[robot_number]
        self.current_vel = np.zeros(2)
        #self.camera_est_pose = np.array([0.,0.,0.])

        # create all subscribers, timers, publishers, listeners, broadcasters,
        # etc.
        self.br = tf.TransformBroadcaster()
        self.pos_pub = rospy.Publisher("pose_est", multi_poses, queue_size=10)
        self.rate = rospy.Rate(50)
        self.cmd_sub = rospy.Subscriber("cmd_vel", multi_vels, self.integrate_callback)
        # self.int_timer = rospy.Timer(rospy.Duration(1/float(self.freq)), self.integrate_callback)
        return

    def integrate_callback(self, tdat):
        v = self.current_vel[0]
        w = self.current_vel[1]

        self.current_pose += (1/float(self.freq))*np.array([
            v*np.cos(self.current_pose[2]),
            v*np.sin(self.current_pose[2]),
            w])

        #publish estimated pose to diffdrive_velocity controller via pose_est topic
        p_est = self.p_est
        p_est.robot_name = self.robot_name
        robot_number = int(p_est.robot_name[6])

        rospy.sleep(1)

        print "robot_number", robot_number
        print "p_est.pose_ests[robot_number].x", p_est.pose_ests[robot_number].x
        print "self.current_pose[0]",self.current_pose[0]

        p_est.pose_ests[robot_number].x = self.current_pose[0]
        p_est.pose_ests[robot_number].y = self.current_pose[1]
        p_est.pose_ests[robot_number].theta = self.current_pose[2]

        self.pos_pub.publish(p_est)

        # now let's send that pose data out as a tf message:
        pos, quat = self.pose_arr_to_tf(self.current_pose)
        self.br.sendTransform(pos, quat, rospy.Time.now(), self.child, self.parent)

        return

    def twist_callback(self, command):
        self.current_vel = self.twist_to_vel_arr(command)
        return

    def pose_arr_to_tf(self, pose):
        pos = [pose[0], pose[1], 0]
        quat = tr.quaternion_from_euler(0, 0, pose[2], 'sxyz')
        return pos, quat

    def twist_to_vel_arr(self, multi):
        num = int(multi.robot_name[6])
        vel = np.array([multi.twists[num].linear.x, multi.twists[num].angular.z])
        return vel


def main():
    #rospy.init_node('diff_drive_integrator', log_level=rospy.INFO)
    rospy.init_node('test_diff_drive_integrator')

    #INITIAL_POSES = np.array([[-2.,0.,0.],[-1.,0.,pi/4.],[0.,0.,pi/2.]])

    # N = number of robots
    N = len(INITIAL_POSES)

    #specify i as index for each robot, have the member functions return the position, then can call the output of that function in main to get the value, which will be appended to array and published/subscribed

    try:
        for i in range(N):
            diffdrive = DiffDriveIntegrator(i)


    except rospy.ROSInterruptException:
        pass

    rospy.spin()


if __name__=='__main__':
    main()

