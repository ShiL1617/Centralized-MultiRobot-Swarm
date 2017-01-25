#!/usr/bin/env python
import rospy
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Pose2D
from geometry_msgs.msg import Twist, Vector3
from math import pi, atan, sin, cos

def camera_noise(Pose2D):
    """
    introduce a noise distribution, for now Gaussian with
    mu = (x,y), sigma = 0.05, and assume number of sample points = 10
    -first take in actual value (x,y,theta)
    -introduce noise to generate 10 points normally distributed around "true" value
    -randomly pick one point to be output (x,y,theta)
    """
    x = Pose2D.x
    y = Pose2D.y
    theta = Pose2D.theta

    x_noise = (np.random.normal(x,0.05,10)).tolist()
    y_noise = (np.random.normal(y,0.05,10)).tolist()

    index = np.random.randint(10)
    x_est = x_noise(index)
    y_est = y_noise(index)

    cf_pose = Pose2D()
    cf_pose.x = x_est
    cf_pose.y = y_est
    cf_pose.theta = theta
    pub.publish(cf_pose)

    return

if __name__=='__main__':

    rospy.init_node('camera_sim')
    pub = rospy.Publisher('pose_update', Pose2D, queue_size=10)
    rospy.Subscriber('vel_update', Pose2D, camera_noise)
    try:
        rospy.spin()
    except KeyboardInterrupt:
        print("Shutting down")
