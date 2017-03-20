#!/usr/bin/env python

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose2D
from winter_project.msg import multi_poses, multi_vels
import tf

#NON ROS IMPORTS
from math import pi, atan, sin, cos, tan, atan2, floor, ceil, sqrt, exp, fmod
import numpy as np


N = 10
BASE_NAME = "reference_"
VELOCITY_TRANSMISSION_FREQ = 250 # Hz
TRAJECTORY_FREQ = 250 # Hz

# circle arrangement for 10 robots
# GOAL_POSES = np.array([[3., 0., 0.]
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
GOAL_POSES = np.array([[-1., 0., 0.],[1., 0., 0.],
    [0.5*sqrt(2.)/2.,0.5*sqrt(2.)/2.,0.],
    [0., 1., 0.],
    [-0.5*sqrt(2.)/2.,0.5*sqrt(2.)/2.,0.],
    [1.*cos(4.*pi/3.), 1*sin(4.*pi/3.), 0.],
    [0.5*cos(7.*pi/6.), 0.5*sin(7.*pi/6.), 0.],
    [0., -0.5, 0.],
    [1.*cos(5.*pi/3), 1.*sin(5.*pi/3.), 0.],
    [0.5*cos(11.*pi/6.), 0.5*sin(11.*pi/6.), 0.]])

# cross arrangement for 10 robots
# GOAL_POSES = np.array([[0., 1., 0.],[0., 0.5, 0.],[-1., 0., 0.],[-0.5, 0., 0.],[0., 0., 0.],[0.5, 0., 0.],[1., 0., 0.],[0., -0.5, 0.],[0., -1., 0.],[0., -2., 0.]])

# GOAL_POSES = np.array([[3., -3., 0.],[-3., -3., 0.]])

# normalizes angle to be 0 to 2*pi
def normalize_angle_positive(theta):
    """ normalizes angle to be 0 to 2*pi """
    return fmod(fmod(float(theta), 2.0*pi) + 2.0*pi, 2.0*pi)

# normalizes to be -pi to pi
def normalize_angle(theta):
    wrapped_angle = normalize_angle_positive(float(theta))
    if wrapped_angle > pi:
        wrapped_angle -= 2.0 * pi
    return wrapped_angle

# returns shortest angular difference between -pi to pi, adding the result to FROM (theta1) will always get equivalent angle TO (theta2)
def shortest_angular_distance(theta1, theta2):
    return normalize_angle(float(theta2) - float(theta1))


#create class
class DiffDriveVelocityController( object ):
    def __init__(self):

        # miscalleneous class variables/constants

        self.K1P = 4.5
        self.K2P = 4.5

        self.gamma = 0.9
        self.b = 4.0
        self.phase_multiplier = 1.
        self.linear_velocity_multiplier = 1.
        self.angular_velocity_multiplier = 1.
        self.bot_radius = 0.2
        self.angularv_shift = 1
        self.freq = rospy.get_param("~freq", VELOCITY_TRANSMISSION_FREQ)
        self.traj_freq = rospy.get_param("~freq", TRAJECTORY_FREQ)
        # self.gamma = 0.001
        # self.b = 0.001

        # CROSS FORMATION
        self.goal_pose = GOAL_POSES

        # self.goal_pose = np.array([[0., 0., 0.]])

        self.v_traj = np.zeros(N)
        self.w_traj = np.zeros(N)

        self.multi_vels = multi_vels()
        self.traj = multi_poses()

        self.bot_poses = np.zeros((N, 3))

        #initialize publishers and subscribers
        self.br = tf.TransformBroadcaster()
        self.pub = rospy.Publisher('cmd_vel', multi_vels, queue_size=10)
        self.t0 = rospy.get_time()
        self.sub = rospy.Subscriber('pose_est', multi_poses, self.MultiPoses_callback)
        self.int_timer = rospy.Timer(rospy.Duration(1/float(self.freq)), self.pos_array_callback)
        # self.trajectory_timer = rospy.Timer(rospy.Duration(1/float(self.traj_freq)), self.trajectorygen_callback)
        return

    def pos_array_callback(self, tdata):

        self.multi_vels = multi_vels()

        # self.goal_pose, self.v_traj, self.w_traj = self.trajectorygen_callback()

        for i in range(N):

            # new control law with trajectory following -incorporating feedforward terms

            # define required variables (current pose, desired pose, desired velocities)

            pose2Dx = self.bot_poses[i,0]
            pose2Dy = self.bot_poses[i,1]
            pose2Dtheta = self.bot_poses[i,2]

            otherbots = np.delete(self.bot_poses,i,0)

            goal_pose_x = self.goal_pose[i][0]
            goal_pose_y = self.goal_pose[i][1]
            goal_pose_theta = self.goal_pose[i][2]

            # feedback proportonal control
            linear_error = ((goal_pose_x-pose2Dx)**2+(goal_pose_y-pose2Dy)**2)**(0.5)

            #applying mod 2*pi to unwrap the current pose estimate orientation
            global_init_theta = ((pose2Dtheta))%(2.*pi)
            global_goal_theta = atan2(goal_pose_y-pose2Dy,goal_pose_x-pose2Dx)%(2.*pi)

            a_first= shortest_angular_distance(global_init_theta, global_goal_theta)
            a_final = shortest_angular_distance(pose2Dtheta, goal_pose_theta)

            # print "a_first =", a_first, "r_first =", r_first

            if abs(a_first) > (pi/50.) and abs(linear_error)>0.01:
                v_control = 0.
                w_control = self.K2P*a_first
            elif abs(linear_error) > 0.01:

                v_control = self.K1P*linear_error
                w_control = 0.

                # insert collision detection/avoidance
                vmax = 4.
                wmax = 4.

                pose_predict = np.array([pose2Dx + vmax*(1/250),pose2Dy + vmax*(1/250),pose2Dtheta + wmax*(1/250)])

                # define "neighborhood" of current robot
                current_x_max = pose_predict[0] + self.bot_radius
                current_x_min = pose_predict[0] - self.bot_radius
                current_y_max = pose_predict[1] + self.bot_radius
                current_y_min = pose_predict[1] - self.bot_radius

                if any(x >= current_x_min and x <= current_x_max  for x in otherbots[:,0]) and any(y >= current_y_min and y <= current_y_max for y in otherbots[:,1]):
                    w_control = pi/6.
                    self.linear_velocity_multiplier = 0.1
                    # self.phase_multiplier = 10.
                    self.orient_offset = -pi/3.
                    self.angularv_shift = -1
                else:
                    w_control = 0;
                    self.linear_velocity_multiplier = 1.
                    # self.phase_multiplier = 1.
                    self.orient_offset = 0.
                    self.angularv_shift = 1

                v_control = self.linear_velocity_multiplier*v_control
                w_control = self.angular_velocity_multiplier*(w_control)

            elif abs(a_final) > pi/50.:
                v_control = 0.
                w_control = self.K2P*a_final

            else:
                v_control = 0.
                w_control = 0.

            print "v_control = ", v_control
            print "w_control = ", w_control

            self.multi_vels.twists.append(Twist(Vector3(v_control,0.,0.), Vector3(0.,0.,w_control)))

            # send transforms for reference:
            # q = tf.transformations.quaternion_from_euler(theta_d, 0, 0, 'szyx')
            # pos = (x_d, y_d, 0)
            # frame = BASE_NAME + str(i)
            # self.br.sendTransform(pos, q, rospy.Time.now(), frame, "odom")


        self.pub.publish(self.multi_vels)
        return

    # def trajectorygen_callback(self):

    #     traj = np.zeros((N,3))
    #     v_traj = np.zeros(N)
    #     w_traj = np.zeros(N)

    #     t = (rospy.get_time() - self.t0)
    #     # print "time =", t
    #     for i in range(N):

    #         #pseudo leader-follower conditions
    #         phase_var = -0.6*(self.phase_multiplier)*i
    #         # phase_var = 0
    #         # vel_adjust = i/1.1

    #         # set radii of each robot's circular trajectory- hardcoded for now
    #         A = 3.

    #         # t = 0.1*t
    #         x_t = A*cos(t+phase_var)
    #         y_t = A*sin(t+phase_var)

    #         dx_dt = -A*sin(t+phase_var)
    #         dy_dt = A*cos(t+phase_var)

    #         d2x_dt2 = -A*cos(t+phase_var)
    #         d2y_dt2 = -A*sin(t+phase_var)

    #         vel_d = sqrt(dx_dt**2 + dy_dt**2)

    #         theta_t = atan2(dy_dt,dx_dt)

    #         dtheta_dt = self.angularv_shift*((d2y_dt2*dx_dt)-(d2x_dt2*dy_dt))/(vel_d)**2

    #         traj[i] = [x_t, y_t, theta_t]
    #         v_traj[i] = vel_d
    #         w_traj[i] = dtheta_dt

    #     return traj, v_traj, w_traj

    def MultiPoses_callback(self, nps):
        bots_poses = np.zeros((N,3))

        for j in range(N):
            bots_poses[j,0] = nps.poses[j].x
            bots_poses[j,1] = nps.poses[j].y
            bots_poses[j,2] = nps.poses[j].theta

        self.bot_poses = bots_poses
        return

def main():
    rospy.init_node('test_diff_drive_velocity_controller')

    try:
        DiffDriveVelocityController()

    except rospy.ROSInterruptException:
        pass

    rospy.spin()

if __name__=='__main__':
    main()
