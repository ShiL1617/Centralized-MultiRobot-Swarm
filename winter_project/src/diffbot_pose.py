#!/usr/bin/env python
import rospy
import tf
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Twist, Vector3, Pose2D, Point, Pose, Quaternion
from math import pi, atan, sin, cos
from nav_msgs.msg import Odometry

#service call?
#import TeleportAbsolute from somewhere else
#hold off on this for now

#maybe use JointState rather than Twist?, use Twist for now

#set up message fields/topics

def integrated_pose(Twist):
    '''
    -using set time step, integrate the kinematics of the differential
    drive robot to derive pose estimate (introduce noise later)
    -method used: Euler integration
    -converts velocities into pose estimates
    '''
    #starting configuration
    pose_x = 0
    pose_y = 0
    pose_theta = pi/2

    #constants
    r = 0.046
    L = 0.16
    dt = 0.1

    x_velocity = (((Twist.linear.x)**2+(Twist.linear.y)**2)**(0.5)*cos(Twist.angular.z))

    y_velocity = (((Twist.linear.x)**2+(Twist.linear.y)**2)**(0.5)*sin(Twist.angular.z))

    theta_velocity = Twist.angular.z

    current_time = rospy.Time.now()
    last_time = rospy.Time.now()
    dt = (current_time-last_time).to_sec()

    pose_x = pose_x + x_velocity*dt
    pose_y = pose_y + y_velocity*dt
    pose_theta = pose_theta + theta_velocity*dt

    pose_new = Pose2D()
    pose_new.x = pose_x
    pose_new.y = pose_y
    pose_new.theta = pose_theta

    pub1.publish(pose_new)
    """
    pose_x = pose_new.x
    pose_y = pose_new.y
    pose_theta = pose_new.theta
    """
    #converting diff bot center axle coordinates to wheel coordinates(to get position and wheel velocities to publish to '/joint_states', may need to initialize another publisher node 'joint_state_publisher')
    jointstate = JointState()

    jointstate.header.stamp = rospy.Time.now()
    jointstate.name = ["lwheel_vel","rwheel_vel"]

    jointstate.velocity = [Twist.linear.x/(r*cos(pose_theta))+L*(Twist.angular.z)/(2*r),(Twist.linear.x)/(r*cos(pose_theta))+L*(Twist.angular.z)/(2*r)+L*(Twist.angular.z)/r]

    jointstate.position = [r*Twist.linear.x/(r*cos(pose_theta))+L*(Twist.angular.z)/(2*r)*dt, r*(Twist.linear.x)/(r*cos(pose_theta))+L*(Twist.angular.z)/(2*r)+L*(Twist.angular.z)/r*dt]

    pub2.publish(jointstate)
    """
    #odometry

    # quaternion conversion
    odom_quat = tf.transformations.quaternion_from_euler(0, 0, pose_theta)

    # publish transform over tf
    odom_broadcaster.sendTransform(
        (pose_x, pose_y, 0.),
        odom_quat,
        current_time,
        "base_link",
        "odom"
    )

    # next, we'll publish the odometry message over ROS
    odom = Odometry()
    odom.header.stamp = current_time
    odom.header.frame_id = "odom"

    # set the position
    odom.pose.pose = Pose(Point(pose_x, pose_y, 0.), Quaternion(*odom_quat))

    # set the velocity
    odom.child_frame_id = "base_link"
    odom.twist.twist = Twist(Vector3(x_velocity, y_velocity, 0), Vector3(0, 0, theta_velocity))

    # publish the message
    odom_pub.publish(odom)

    last_time = current_time
    """
    return

if __name__=='__main__':

    #initializations
    rospy.init_node('diffbot_pose')
    """
    odom_pub = rospy.Publisher('/odom', Odometry, queue_size=50)
    odom_broadcaster = tf.TransformBroadcaster()
    """
    pub1 = rospy.Publisher('/camera_pose', Pose2D, queue_size=10)
    pub2 = rospy.Publisher('/joint_states', JointState, queue_size=10)
    rate = rospy.Rate(50)
    rospy.Subscriber('/cmd_vel', Twist, integrated_pose)

    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
