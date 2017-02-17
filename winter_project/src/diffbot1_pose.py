#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Pose2D, Point, Pose, Quaternion

from math import pi, atan, sin, cos

rospy.init_node('vel_pose_est1')
pub = rospy.Publisher('/pose_est', Pose2D, queue_size=10)
rate = rospy.Rate(100)

odom_broadcaster = tf.TransformBroadcaster()

#specify starting pose and goal pose
#start
pose_x = 0
pose_y = 0
pose_theta = 0

#goal
x_final = 0
y_final = 2
theta_final = 0

#velocity control law and gains

#gains K1, K2
K1 = 0.1
K2 = 0.1

#control law
vel = Twist()

#start loop

while not rospy.is_shutdown():
    #make sure this error term is correct?
    linear_error = ((x_final-pose_x)**2+(y_final-pose_y)**2)**(0.5)

    #need to add in angle wrapping correction
    angular_error = abs(theta_final-(pi/2-(pose_theta%(2*pi))))

    vel.linear.x = (K1*linear_error*cos(angular_error))*cos(pose_theta)
    vel.linear.y = (K1*linear_error*cos(angular_error))*sin(pose_theta)
    vel.linear.z = 0

    vel.angular.x = 0
    vel.angular.y = 0
    vel.angular.z = K1*sin(angular_error)*cos(angular_error) + K2*angular_error

    #Euler integration- find integrated position and orientation, make sure in right coordinate system (v,omega) with respect to how robot actually moves
    v = ((vel.linear.x)**2+(vel.linear.y)**2)**(0.5)

    w = vel.angular.z

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    dt = (current_time-last_time).to_sec()

    #insert integration noise here later (no encoders)

    pose_x = pose_x + v*cos(pose_theta)*(1/float(100))
    pose_y = pose_y + v*sin(pose_theta)*(1/float(100))
    pose_theta = pose_theta + w*(1/float(100))

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
        "base_link",
        "odom"
    )

    last_time = current_time
    rate.sleep()


