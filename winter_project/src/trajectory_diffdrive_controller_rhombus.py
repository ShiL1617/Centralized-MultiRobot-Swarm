#!/usr/bin/env python

#ROS IMPORTS
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose2D
from winter_project.msg import multi_poses, multi_vels
import tf
import tf2_ros
from visualization_msgs.msg import Marker

#NON ROS IMPORTS
from math import pi, atan, sin, cos, tan, atan2, floor, ceil, sqrt, exp, fmod
import numpy as np
import matplotlib.pyplot as plt



#GLOBAL CONSTANTS
#CONTROLLER_NUM = 0
# GOAL_POSES = np.array([[1.,1.,pi/2.]])
# N = len(GOAL_POSES) # number of robots
N = 3
BASE_NAME = "reference_"
VELOCITY_TRANSMISSION_FREQ = 100 # Hz
TRAJECTORY_SAMPLE_RATE = 0.05 # seconds

MAXIMUM_LINEAR_VELOCITY = 10.
MAXIMUM_ANGULAR_VELOCITY = 10.

# normalizes angle to be between 0 and pi/2
def normalize_angle_half_pi(theta):
    """ normalizes angle to be 0 to pi/2 """
    return fmod(fmod(float(theta), pi/2.0) + 2.0*pi, 2.0*pi)

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

# returns distances between each robots, specifically an array of size N-1 x 1 for N robots
def inter_bot_distance(bots_position):
    x = bots_position[:,0]
    y = bots_position[:,1]
    inter_dists = []
    for i in range(N-1):
        a = np.array([x[i], y[i]])
        b = np.array([x[i+1], y[i+1]])
        inter_dists.append(np.linalg.norm((a-b),axis=0))
    return np.array(inter_dists)

#create class
class DiffDriveVelocityController( object ):
    def __init__(self):

        # miscalleneous class variables/constants
        self.gamma = 0.9
        self.b = 4.0
        self.phase_multiplier = 1.
        self.linear_velocity_multiplier = 1.
        self.angular_velocity_multiplier = 1.
        self.bot_radius = 0.02
        self.angularv_shift = 1
        self.freq = rospy.get_param("~freq", VELOCITY_TRANSMISSION_FREQ)
        self.traj_sample = rospy.get_param("~traj_sample", TRAJECTORY_SAMPLE_RATE)
        self.vmax = rospy.get_param("~vmax",MAXIMUM_LINEAR_VELOCITY)
        self.wmax = rospy.get_param("~wmax",MAXIMUM_ANGULAR_VELOCITY)
        self.marker_count = 0
        self.marker_trajectory = Marker()
        # self.gamma = 0.001
        # self.b = 0.001

        #hardcoded goal poses
        self.goal_pose = np.zeros((N,3))
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

        self.marker_pub = rospy.Publisher('/visualization_marker', Marker, queue_size=10)

        self.frameBuffer = tf2_ros.Buffer()
        self.frames_listener = tf2_ros.TransformListener(self.frameBuffer)
        # self.trajectory_timer = rospy.Timer(rospy.Duration(1/float(self.traj_freq)), self.trajectorygen_callback)
        return

    def pos_array_callback(self, tdata):

        self.multi_vels = multi_vels()

        self.goal_pose, self.v_traj, self.w_traj = self.trajectorygen_callback()

        for i in range(N):

            # new control law with trajectory following -incorporating feedforward terms

            # distances between every robot for future calculations related to automatic trajectory distributing\

            total_bot_separation = sum(inter_bot_distance(self.bot_poses))

            print "total_bot_separation =", total_bot_separation

            if total_bot_separation < 0.32:
                self.phase_multiplier = 1.1
            else:
                self.phase_multiplier = 1.0

            # print "bot_inter_distances =", bot_inter_distances

            # define required variables (current pose, desired pose, desired velocities)
            x = self.bot_poses[i,0]
            y = self.bot_poses[i,1]
            theta = self.bot_poses[i,2]

            otherbots = np.delete(self.bot_poses,i,0)

            x_d = self.goal_pose[i][0]
            y_d = self.goal_pose[i][1]
            theta_d = self.goal_pose[i][2]

            v_d = self.v_traj[i] # desired linear velocity
            w_d = self.w_traj[i] # desired angular velocity

            #difference terms
            x_diff = x_d - x
            y_diff = y_d - y
            theta_diff = shortest_angular_distance(theta, theta_d)

            #define control gain functions
            self.k1 = 2. * self.gamma * sqrt((w_d)**2 + self.b * (v_d)**2)
            self.k2_bar = self.b
            self.k3 = 2. * self.gamma * sqrt((w_d)**2 + self.b * (v_d)**2)

            v_control = v_d * cos(theta_diff) + self.k1 * (cos(theta)*x_diff + sin(theta)*y_diff)
            w_control = w_d + self.k2_bar * v_d * ((sin(theta_diff))/(theta_diff))*(cos(theta) * (x_diff) - sin(theta) * (y_diff)) + self.k3 * (theta_diff)

            # collision detection scheme (rudimentary)

            # constrain forward/backward positive/negative linear and angular velocities to within the specified velocity limits
            if v_control > self.vmax:
                v_control = self.vmax
            elif w_control > self.wmax:
                w_control = self.wmax
            elif v_control < -(self.vmax):
                v_control = -(self.vmax)
            elif w_control < -(self.wmax):
                w_control = -(self.wmax)

            # print "w_control for robot #", i, "=", w_control

            # predict based on velocity*time what future robot pose is, see if there are other robots within its "safety" box

            if normalize_angle_positive(theta) >= 0 and normalize_angle_positive(theta) < pi/2.:

                pose_predict = np.array([x + (self.vmax*(1/float(self.freq)))*cos(theta), y + (self.vmax*(1/float(self.freq)))*sin(theta), theta + self.wmax*(1/float(self.freq))])

            elif normalize_angle_positive(theta) >= pi/2. and normalize_angle_positive(theta) < pi:

                pose_predict = np.array([x - (self.vmax*(1/float(self.freq)))*cos(theta), y + (self.vmax*(1/float(self.freq)))*sin(theta), theta + self.wmax*(1/float(self.freq))])

            elif normalize_angle_positive(theta) >= pi and normalize_angle_positive(theta) < 3.*pi/2.:

                pose_predict = np.array([x - (self.vmax*(1/float(self.freq)))*cos(theta), y - (self.vmax*(1/float(self.freq)))*sin(theta), theta + self.wmax*(1/float(self.freq))])

            elif normalize_angle_positive(theta) >= 3.*pi/2. and normalize_angle_positive(theta) < 2.*pi:

                pose_predict = np.array([x + (self.vmax*(1/float(self.freq)))*cos(theta), y + (self.vmax*(1/float(self.freq)))*sin(theta), theta + self.wmax*(1/float(self.freq))])

            # define "neighborhood" of current robot
            current_x_max = pose_predict[0] + self.bot_radius
            current_x_min = pose_predict[0] - self.bot_radius
            current_y_max = pose_predict[1] + self.bot_radius
            current_y_min = pose_predict[1] - self.bot_radius

            if any(x >= current_x_min and x <= current_x_max  for x in otherbots[:,0]) and any(y >= current_y_min and y <= current_y_max for y in otherbots[:,1]):
                self.angular_velocity_multiplier = 1.1
                self.linear_velocity_multiplier = 0.1

            else:
                self.angular_velocity_multiplier = 1.
                self.linear_velocity_multiplier = 1.

            v_control = self.linear_velocity_multiplier*v_control
            w_control = self.angular_velocity_multiplier*w_control

            # automatic trajectory distributing for "bunching" situation

            # check for velocity decrease at points where dy/dt and dx/dt w.r.t. fixed frame are small enough by set criteria
            # if v_d < 0.1:
            #     self.phase_multiplier = 1.5

            # if fixed distance between each robot is broken, adjust phase multiplier to preserve notion of constant distance between robot


            # print "v_d =", v_d
            print "phase_multiplier =", self.phase_multiplier

            self.multi_vels.twists.append(Twist(Vector3(v_control,0.,0.), Vector3(0.,0.,w_control)))

            # send transforms for reference, set Marker visualization:

            q = tf.transformations.quaternion_from_euler(theta_d, 0, 0, 'szyx')
            pos = (x_d, y_d, 0)
            frame = BASE_NAME + str(i)
            ref_frame = BASE_NAME + str(0)
            self.br.sendTransform(pos, q, rospy.Time.now(), frame, "odom")

            try:
                t = self.frameBuffer.lookup_transform("odom", ref_frame, rospy.Time(0))

            except(tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):

                continue

            self.marker_count += 1

            self.marker_trajectory.id = self.marker_count
            self.marker_trajectory.header.frame_id = "odom"
            self.marker_trajectory.header.stamp = rospy.Time.now()
            self.marker_trajectory.action = 0

            self.marker_trajectory.pose.position.x = t.transform.translation.x
            self.marker_trajectory.pose.position.y = t.transform.translation.y
            self.marker_trajectory.pose.position.z = t.transform.translation.z

            # print marker_trajectory.pose.orientation.x

            self.marker_trajectory.type = 2 #Sphere
            self.marker_trajectory.color.r = 1.0
            self.marker_trajectory.color.g = 1.0
            self.marker_trajectory.color.b = 1.0
            self.marker_trajectory.color.a = 0.5
            self.marker_trajectory.scale.x = 0.05
            self.marker_trajectory.scale.y = 0.05
            self.marker_trajectory.scale.z = 0.05

            self.marker_trajectory.lifetime = rospy.Duration(2.5)

            self.marker_pub.publish(self.marker_trajectory)

        self.pub.publish(self.multi_vels)
        return

    def trajectorygen_callback(self):

        # temporary solution- set rospy.sleep for duration set by trajectory sampling rate so trajectory isn't sampled as often- TODO: change this functionality to rospy Timer()
        rospy.sleep(TRAJECTORY_SAMPLE_RATE)

        traj = np.zeros((N,3))
        v_traj = np.zeros(N)
        w_traj = np.zeros(N)

        t = (rospy.get_time() - self.t0)
        # print "time =", t
        for i in range(N):

            #pseudo leader-follower conditions
            phase_var = -0.06*(self.phase_multiplier)*i
            # phase_var = 0
            # vel_adjust = i/1.1

            # set radii of each robot's circular trajectory- hardcoded for now
            # A = 1.

            # # t = 0.1*t
            # x_t = A*cos(t+phase_var)
            # y_t = A*2.5*sin(t+phase_var)

            # dx_dt = -A*sin(t+phase_var)
            # dy_dt = 2.5*A*cos(t+phase_var)

            # d2x_dt2 = -A*cos(t+phase_var)
            # d2y_dt2 = -2.5*A*sin(t+phase_var)

            # superellipse reference trajectory
            A = 2.5
            B = 1.5
            r = 1.0

            # t = normalize_angle_half_pi(t)
            x_t = A*(cos(t+phase_var))**(2./r)
            y_t = B*(sin(t+phase_var))**(2./r)

            # adjustments for angular velocity errors:
            sgnx = sgny = 1
            theta_adjust = 0

            # traveling full reference trajectory for r <1:
            if r < 1:
                if normalize_angle_positive(t) >= pi/2. and normalize_angle_positive(t) <= (pi):
                    x_t = -x_t
                    sgnx = -1
                if normalize_angle_positive(t) >= pi and normalize_angle_positive(t) <= (3.*pi/2.):
                    x_t = -x_t
                    y_t = -y_t
                    theta_adjust = pi
                if normalize_angle_positive(t) >= (3.*pi/2.) and normalize_angle_positive(t) <= (2*pi):
                    y_t = -y_t
                    sgny = -1

            # traveling full reference trajectory for r <1:
            if r == 1.0:
                if normalize_angle_positive(t) >= pi/2. and normalize_angle_positive(t) <= (pi):
                    x_t = -x_t
                    sgnx = -1
                if normalize_angle_positive(t) >= pi and normalize_angle_positive(t) <= (3.*pi/2.):
                    x_t = -x_t
                    y_t = -y_t
                    theta_adjust = pi
                if normalize_angle_positive(t) >= (3.*pi/2.) and normalize_angle_positive(t) <= (2*pi):
                    y_t = -y_t
                    sgny = -1



            dx_dt = A*(2./r)*cos(t+phase_var)**(2./r-1)*(-sin(t+phase_var))
            dy_dt = B*(2./r)*sin(t+phase_var)**(2./r-1)*(cos(t+phase_var))

            d2x_dt2 = (-2.*A*cos(t+phase_var)**(2./r))/r + (2.*A*(-1 + 2./r)*cos(t+phase_var)**(-2 + 2./r)*sin(t+phase_var)**2)/r
            d2y_dt2 = (2.*B*(-1 + 2./r)*cos(t+phase_var)**2*sin(t+phase_var)**(-2 + 2./r))/r - (2.* B*sin(t+phase_var)**(2./r))/r

            vel_d = sqrt(dx_dt**2 + dy_dt**2)

            theta_t = atan2(sgny*dy_dt,sgnx*dx_dt) + theta_adjust

            dtheta_dt = self.angularv_shift*((d2y_dt2*dx_dt)-(d2x_dt2*dy_dt))/(vel_d)**2

            traj[i] = [x_t, y_t, theta_t]
            v_traj[i] = vel_d
            w_traj[i] = dtheta_dt

        return traj, v_traj, w_traj

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
