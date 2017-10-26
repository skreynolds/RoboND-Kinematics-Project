#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import rospy
import tf
from kuka_arm.srv import *
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from geometry_msgs.msg import Pose
from mpmath import *
from sympy import *


def handle_calculate_IK(req):
    rospy.loginfo("Received %s eef-poses from the plan" % len(req.poses))
    if len(req.poses) < 1:
        print "No valid poses received"
        return -1
    else:
        ### Your FK code here
        # Create symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8')
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8')
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7')
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7')

        # Create Modified DH parameters
        s = {alpha0:     0,    a0:      0,    d1: 0.68,
             alpha1: -pi/2,    a1:   0.42,    d2:    0,    q2: (q2 - pi/2),
             alpha2:     0,    a2:   1.25,    d3:    0}

        # Define Modified DH Transformation matrix
        T0_1 = transMat(q1, alpha0, d1, a0)
        T1_2 = transMat(q2, alpha1, d2, a1)
        T2_3 = transMat(q3, alpha2, d3, a2)

        T0_1 = T0_1.subs(s)
        T1_2 = T1_2.subs(s)
        T2_3 = T2_3.subs(s)

        # Create individual transformation matrices
        T0_2 = T0_1*T1_2
        T0_3 = T0_2*T2_3

        # Specify the intrinsic rotation matrix for correcting from DH to urdf
        R_corr = Matrix([[0,  0, 1],
                         [0, -1, 0],
                         [1,  0, 0]])

        # Extract rotation matrices from the transformation matrices
        R0_3 = T0_3[0:3,0:3]
        ###

        # Initialize service response
        joint_trajectory_list = []
        for x in xrange(0, len(req.poses)):
            # IK code starts here
            joint_trajectory_point = JointTrajectoryPoint()

	        # Extract end-effector position and orientation from request
	        # px,py,pz = end-effector position
	        # roll, pitch, yaw = end-effector orientation
            px = req.poses[x].position.x
            py = req.poses[x].position.y
            pz = req.poses[x].position.z

            (roll, pitch, yaw) = tf.transformations.euler_from_quaternion(
                [req.poses[x].orientation.x, req.poses[x].orientation.y,
                    req.poses[x].orientation.z, req.poses[x].orientation.w])

            ### Your IK code here
	        # Compensate for rotation discrepancy between DH parameters and Gazebo
            Rrpy = R_z(roll) * R_y(pitch) * R_x(yaw) * R_corr
	        #
	        # Calculate joint angles using Geometric IK method
            wx = px - (0.303) * Rrpy[0,2] # x-coord of wrist position
            wy = py - (0.303) * Rrpy[1,2] # y-coord of wrist position
            wz = pz - (0.303) * Rrpy[2,2]# z-coord of wrist position

            r = sqrt(wx**2 + wy**2) - 0.35
            ss = wz - 0.75

            k1 = 1.25
            k2 = sqrt(0.96**2 + 0.054**2)

            D = (r**2 + s**2 - k1**2 - k2**2)/(2*k1*k2)
            K = (k1 + k2*D)/sqrt(r**2 + ss**2)

            theta1 = atan2(wy, wx)
            theta2 = atan2(ss, r) - atan2(sqrt(1 - K**2), K)
            theta3 = atan2(D, sqrt(1 - D**2))

            R36rpy = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3}) * Rrpy

            theta4 = atan(-R36rpy[2,2], R36rpy[0,3])
            theta5 = atan(sqrt(1 - R36rpy[1,2]**2), R36rpy[1,2])
            theta6 = atan(-R36rpy[1,1], R36rpy[1,0])
	        #
            ###

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)


def transMat(q, alpha, d, a):
    T = Matrix([[           cos(q),           -sin(q),           0,             a],
                [sin(q)*cos(alpha), cos(q)*cos(alpha), -sin(alpha), -sin(alpha)*d],
                [sin(q)*sin(alpha), cos(q)*sin(alpha),  cos(alpha),  cos(alpha)*d],
                [                 0,                0,           0,             1]])
    return T

def R_x(q):
    M_x = Matrix([[ 1,      0,       0],
                  [ 0, cos(q), -sin(q)],
                  [ 0, sin(q),  cos(q)]])
    return M_x

def R_y(q):
    M_y = Matrix([[ cos(q), 0, sin(q)],
                  [      0, 1,       0],
                  [-sin(q), 0, cos(q)]])
    return M_y

def R_z(q):
    M_z = Matrix([[ cos(q), -sin(q), 0],
                  [ sin(q),  cos(q), 0],
                  [ 0,            0, 1]])
    return M_z


def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
