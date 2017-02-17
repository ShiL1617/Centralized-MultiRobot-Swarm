#!/usr/bin/env python
import rospy
from geometry_msgs.msg import Twist, Vector3, Pose2D
from math import pi, atan, sin, cos, atan2

"""

#applying mod 2*pi to unwrap the current pose estimate orientation
unwrapped_theta = ((pose2D.theta))%(2*pi)
global_init_theta = unwrapped_theta
global_goal_theta = atan2(y_final-pose2D.y,x_final-pose2D.x)%(2*pi)


    def first_turn(self):

        theta1 = ((self.theta))%(2*pi)
        theta2 = atan2(self.goal_pose[1]-self.y,self.goal_pose[0]-self.x)%(2*pi)

        if abs(theta1 - theta2) >= pi:
            self.global_rotation_error = 2*pi-abs(theta1 - theta2)
        else:
            self.global_rotation_error= abs(theta1 - theta2)

        if theta1 > theta2:
            self.rotation_direction_global = -1
        else:
            self.rotation_direction_global = 1

        return

    def final_turn(self):

        if abs(self.theta-self.goal_pose[2]) >= pi:
            self.body_rotation_error = 2*pi-abs(self.theta-self.goal_pose[2])
        else:
            self.body_rotation_error= abs(self.theta-self.goal_pose[2])

        if self.theta > self.goal_pose[2]:
            self.rotation_direction_body = -1
        else:
            self.rotation_direction_body = 1


global_orientation_error, rd_g = anglediff(global_init_theta, global_goal_theta)

body_orientation_error,rd_b = anglediff(pose2D.theta,theta_final)

"""

def anglediff(theta_current, theta_final):
    #checking all 4 quadrants for each case with goal and current orientation in different quadrants
    if abs(theta_current-theta_final) >= pi:
        angular_error = 2*pi-abs(theta_current-theta_final)
    else:
        angular_error = abs(theta_current-theta_final)

    if theta_current > theta_final:
        rotation_direction = -1
    else:
        rotation_direction = 1

    return angular_error, rotation_direction



def coord2vel(pose2D):
    # count += 1

    #gains K1, K2
    K1p = 3.
    K2p = 3.

    #goal position
    x_final = 0.
    y_final = 2.
    theta_final = pi
    # theta_final = pi/6.

    vel = Twist()

    linear_error = ((x_final-pose2D.x)**2+(y_final-pose2D.y)**2)**(0.5)

    #applying mod 2*pi to unwrap the current pose estimate orientation
    unwrapped_theta = ((pose2D.theta))%(2*pi)
    global_init_theta = unwrapped_theta
    #global_init_theta = atan2(pose2D.y,pose2D.x)%(2*pi)
    global_goal_theta = atan2(y_final-pose2D.y,x_final-pose2D.x)%(2*pi)

    global_orientation_error, rd_g = anglediff(global_init_theta, global_goal_theta)
    body_orientation_error, rd_b = anglediff(global_goal_theta, theta_final)


    body_orientation_error,rd_b = anglediff(pose2D.theta,theta_final)
    # if count > 25:
    # print "WTF IS GOING ON"

    #    final_orientation_error, rd_g = anglediff(atan2(pose2D.y,pose2D.x)%(2*pi), global_goal_theta)

    #control law is P control with K_i*(error_i)

    if abs(global_orientation_error)>pi/100. and abs(linear_error)>0.01:# and abs(body_orientation_error) > pi/100.:
        vel.linear.x = 0.
        vel.angular.z = K2p*global_orientation_error*rd_g
        # rospy.loginfo("First turn!")

    # elif abs(global_orientation_error)<=pi/100. and abs(linear_error)>0.01 and abs(body_orientation_error) > pi/100.:
    elif abs(linear_error)>0.01:
        vel.linear.x = K1p*linear_error
        vel.angular.z = 0.
        # rospy.loginfo("Driving forward!")

    # elif abs(global_orientation_error)<=pi/100. and abs(linear_error)<=0.01 and abs(body_orientation_error) > pi/100.:
    elif abs(body_orientation_error) > pi/100.:
        vel.linear.x = 0.
        vel.angular.z = K2p*0.05*body_orientation_error*rd_b
        # rospy.loginfo("Final Turn!")
        print "body_orientation_error:", body_orientation_error


    else:
        vel.linear.x = 0.
        vel.angular.z = 0.
        rospy.loginfo("Finished!")

        #rospy.loginfo("count = %d",count)
    #global orientation
    """
    if abs(global_orientation_error) > pi/1000.:
        vel.linear.x = 0
        vel.angular.z = K2p*global_orientation_error*rd_g
    else:
        rospy.loginfo("global angular error is %f"%global_orientation_error)
        vel.linear.x = 0
        vel.angular.z = 0
    """
    """
    #linear
    if abs(linear_error) >= 1/100.:

        vel.linear.x = K1p*linear_error
        vel.angular.z = 0
    else:
        rospy.loginfo("linear error is %f"%linear_error)
        vel.linear.x = 0
        vel.angular.z = 0
    """
    """
    #body orientation
    if abs(body_orientation_error)>pi/100.:
        vel.linear.x = 0
        vel.angular.z = K2p*body_orientation_error*rd_b
    else:
        vel.linear.x = 0
        vel.angular.z = 0
    """
    #control law is PI control with K_i*(error_i) and K_i*integral(error_i)*dt
    """
    linear_integral_error.append(linear_error*float(1/250.))
    angular_integral_error.append(angular_error*float(1/250.))

    if angular_error > 0.05*pi and linear_error > 0.02:
        vel.linear.x = 0
        vel.angular.z = K2p*angular_error + K2i*sum(angular_integral_error)
    elif angular_error <= 0.05*pi and linear_error > 0.02:
        vel.linear.x = K1p*linear_error + K1i*sum(linear_integral_error)
        vel.angular.z = 0
    elif angular_error > 0.01*pi and linear_error <= 0.02:
        vel.linear.x = 0
        vel.angular.z = K2p*angular_error + K2i*sum(angular_integral_error)
    else:
        vel.linear.x = 0
        vel.angular.z = 0

    #validation of controller/testing if pub/sub process is communicating correctly
    """
    """
    vel.linear.x = 1
    vel.angular.z = 1
    """
    pub.publish(vel)
    rate.sleep()
    return

if __name__=='__main__':
    rospy.init_node('diff_drive_velocity_controller')
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)
    rate = rospy.Rate(250)

    #pub_error = rospy.Publisher('error_topic', Pose2D, queue_size=10)
    # count = 0
    rospy.Subscriber('pose_est', Pose2D, coord2vel)

    angular_integral_error = []
    linear_integral_error = []

    rospy.spin()
