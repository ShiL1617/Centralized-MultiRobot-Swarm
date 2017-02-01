#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Pose2D, Point, Pose, Quaternion

from math import pi, atan, sin, cos

rospy.init_node('vel_pose_est3')
pub = rospy.Publisher('/pose_est', Pose2D, queue_size=10)
rate = rospy.Rate(100)

odom_broadcaster = tf.TransformBroadcaster()

#robot2

#specify starting pose and goal pose
#start
pose_x = 2
pose_y = 0
pose_theta = 0

#goal
x_final = -1
y_final = 4
theta_final = pi/2

#velocity control law and gains

#gains K1, K2
K1 = 20
K2 = 15

#control law
vel = Twist()

#start loop

#(x_final-pose_x < 0.01 and y_final-pose_y < 0.01 and theta_final-pose_theta < 0.01)

while not rospy.is_shutdown():

    linear_error = ((x_final-pose_x)**2+(y_final-pose_y)**2)**(0.5)
    angular_error = abs(theta_final-pose_theta)

    vel.linear.x = (K1*linear_error*cos(angular_error))*cos(pose_theta)
    vel.linear.y = (K1*linear_error*cos(angular_error))*sin(pose_theta)
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = K1*sin(angular_error)*cos(angular_error) + K2*angular_error

    #Euler integration- find integrated position and orientation
    x_velocity = (((vel.linear.x)**2+(vel.linear.y)**2)**(0.5)*cos(vel.angular.z))

    y_velocity = (((vel.linear.x)**2+(vel.linear.y)**2)**(0.5)*sin(vel.angular.z))

    theta_velocity = vel.angular.z

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    dt = (current_time-last_time).to_sec()

    #insert integration noise here later (no encoders)

    pose_x = pose_x + x_velocity*dt
    pose_y = pose_y + y_velocity*dt
    pose_theta = pose_theta + theta_velocity*dt

    #camera noise here later, pose estimate from here is input to velocity control, then loop starts over

    #publish pose estimate
    pose_updated = Pose2D()

    pose_updated.x = pose_x
    pose_updated.y = pose_y
    pose_updated.theta = pose_theta

    pub.publish(pose_updated)

    #define tf transforms, send transforms with tf.TransformBroadcaster()
    odom_quat = tf.transformations.quaternion_from_euler(0., 0., pose_updated.theta)

    # publish transform over tf
    odom_broadcaster.sendTransform(
        (pose_updated.x, pose_updated.y, 0.),
        odom_quat,
        current_time,
        "robot3/base_link",
        "odom"
    )

    last_time = current_time
    rate.sleep()

