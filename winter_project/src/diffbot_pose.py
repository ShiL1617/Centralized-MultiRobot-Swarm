#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Pose2D, Point, Pose, Quaternion
from math import pi, atan, sin, cos
from nav_msgs.msg import Odometry

def integrated_pose(Twist):
    '''
    -using time step dt, integrate the kinematics of the differential
    drive robot to derive pose estimate (introduce noise later)
    -method used: Euler integration
    -converts velocities into pose estimates
    -publishes tf transforms
    '''
    #constants
    r = 0.046
    L = 0.16
    dt = 0.1

    start_time = rospy.Time.now()

    if start_time.to_sec() < 0.1:
        x_velocity = 0
        y_velocity = 0
        theta_velocity = 0
    else:
        x_velocity = (((Twist.linear.x)**2+(Twist.linear.y)**2)**(0.5)*cos(Twist.angular.z))

        y_velocity = (((Twist.linear.x)**2+(Twist.linear.y)**2)**(0.5)*sin(Twist.angular.z))

        theta_velocity = Twist.angular.z

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    dt = (current_time-last_time).to_sec()
    """
    if current_time.to_sec() < 0.01:
    #starting configuration
        pose_x = 0
        pose_y = 0
        pose_theta = pi/2
    """
    pose_x = pose_x + x_velocity*dt
    pose_y = pose_y + y_velocity*dt
    pose_theta = pose_theta + theta_velocity*dt

    pose_new = Pose2D()
    pose_new.x = pose_x
    pose_new.y = pose_y
    pose_new.theta = pose_theta

    pub.publish(pose_new)

    pose_x = pose_new.x
    pose_y = pose_new.y
    pose_theta = pose_new.theta

    # quaternion conversion
    odom_quat = tf.transformations.quaternion_from_euler(0., 0., pose_theta)

    # publish transform over tf
    odom_broadcaster.sendTransform(
        (pose_x, pose_y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    """
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "/odom"

    # set the position
    odom.pose.pose = Pose(Point(pose_x, pose_y, 0.), odom_quat)

    # set the velocity
    odom.child_frame_id = "/base_link"

    odom.twist.twist.linear.x  = x_velocity
    odom.twist.twist.linear.y  = y_velocity
    odom.twist.twist.linear.z  = 0

    odom.twist.twist.angular.x  = 0
    odom.twist.twist.angular.y  = 0
    odom.twist.twist.angular.z  =  theta_velocity

    odom_pub.publish(odom)
    """
    last_time = current_time
    rate.sleep()

    return

def pose_pub():

    # p_publish = rospy.Publisher('/camera_pose',Pose2D, queue_size=10)
    rospy.Subscriber('/cmd_vel', Twist, integrated_pose)

    pose_est = Pose2D()

    pose_est.x = 0
    pose_est.y = 0
    pose_est.theta = pi/2

    while not rospy.is_shutdown():
        p_publish.publish(pose_start)

    return

if __name__=='__main__':

    #initializations
    rospy.init_node('diffbot_pose')
    pub = rospy.Publisher('/camera_pose', Pose2D, queue_size=10)
    rate = rospy.Rate(50)

    odom_broadcaster = tf.TransformBroadcaster()

    #start publishing initial pose estimate
    pose_pub()


    #function_for_publishing()

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
