#!/usr/bin/env python

# Copyright (C) 2017 Electric Movement Inc.
#
# This file is part of Robotic Arm: Pick and Place project for Udacity
# Robotics nano-degree program
#
# All Rights Reserved.

# Author: Harsh Pandya

# import modules
import numpy as np
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

        # joint angle symbols
        q1, q2, q3, q4, q5, q6, q7 = symbols('q1:8') # theta_i
        # DH param symbols
        d1, d2, d3, d4, d5, d6, d7 = symbols('d1:8') # link offset
        a0, a1, a2, a3, a4, a5, a6 = symbols('a0:7') # link length
        alpha0, alpha1, alpha2, alpha3, alpha4, alpha5, alpha6 = symbols('alpha0:7') # twist angle

        # Modified DH parameters KR210 forward kinematics sections
    	DH_table = {alpha0:       0, a0:      0, d1:  0.75, q1:           q1,
                    alpha1: -pi/2.0, a1:   0.35, d2:     0, q2: -pi/2.0 + q2,
                    alpha2:       0, a2:   1.25, d3:     0, q3:           q3,
                    alpha3: -pi/2.0, a3: -0.054, d4:   1.5, q4:           q4,
                    alpha4:  pi/2.0, a4:      0, d5:     0, q5:           q5,
                    alpha5: -pi/2.0, a5:      0, d6:     0, q6:           q6,
                    alpha6:       0, a6:      0, d7: 0.303, q7:            0}

        # Define Modified DH Transformation matrix
        def TF_matrix(alpha, a, d, q):
            TF = Matrix([[            cos(q),           -sin(q),           0,             a],
                         [ cos(alpha)*sin(q), cos(alpha)*cos(q), -sin(alpha), -sin(alpha)*d],
                         [ sin(alpha)*sin(q), sin(alpha)*cos(q),  cos(alpha),  cos(alpha)*d],
                         [                 0,                 0,           0,             1]])
            return TF

    	# Create individual transformation matrices
    	T0_1 = TF_matrix(alpha0, a0, d1, q1).subs(DH_table)
        T1_2 = TF_matrix(alpha1, a1, d2, q2).subs(DH_table)
        T2_3 = TF_matrix(alpha2, a2, d3, q3).subs(DH_table)
        T3_4 = TF_matrix(alpha3, a3, d4, q4).subs(DH_table)
        T4_5 = TF_matrix(alpha4, a4, d5, q5).subs(DH_table)
        T5_6 = TF_matrix(alpha5, a5, d6, q6).subs(DH_table)
        T6_EE = TF_matrix(alpha6, a6, d7, q7).subs(DH_table)

    	T0EE = simplify(T0_1 * T1_2 * T2_3 * T3_4 * T4_5 * T5_6 * T6_EE)
        T0_3 = T0_1 * T1_2 * T2_3

    	# Extract rotation matrices from the transformation matrices
        def Rotation_matrix(r, p, y):

            Rot_x = Matrix([[   1,      0,       0],
                            [   0, cos(r), -sin(r)],
                            [   0, sin(r),  cos(r)]]) # Roll

            Rot_y = Matrix([[ cos(p),  0, sin(p)],
                            [      0,  1,      0],
                            [-sin(p),  0, cos(p)]]) # Pitch

            Rot_z = Matrix([[ cos(y), -sin(y),  0],
                            [ sin(y),  cos(y),  0],
                            [      0,       0,  1]]) # Yaw

            ROT_EE = Rot_z * Rot_y * Rot_x
            Rot_corr = Rot_z.subs(y, np.pi) * Rot_y.subs(p, -np.pi/2.0)
            ROT_EE = ROT_EE * Rot_corr

            return ROT_EE

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
            # More Info in KR210 foeward Kinematics section
            r, p, y = symbols('r p y')
            ROT_EE = Rotation_matrix(r, p, y).subs({'r': roll, 'p': pitch, 'y': yaw})

            EE = Matrix([[px], [py], [pz]])
            WC = EE - 0.303 * ROT_EE[:,2]

    	    # Calculate joint angles using Geometric IK method
            theta1 = atan2(WC[1], WC[0])

            side_a = 1.501
            side_b = sqrt((sqrt(WC[0]**2 + WC[1]**2) - 0.35)**2 + (WC[2] - 0.75)**2)
            side_c = 1.25

            angle_a = acos((side_b**2 + side_c**2 - side_a**2) / (2 * side_b * side_c))
            angle_b = acos((side_a**2 + side_c**2 - side_b**2) / (2 * side_a * side_c))

            theta2 = pi/2 - angle_a - atan2(WC[2] - 0.75, sqrt(WC[0]**2 + WC[1]**2) - 0.35)
            theta3 = pi/2 -(angle_b + 0.036)

            R0_3 = T0_3[:3,:3]
            R0_3 = R0_3.evalf(subs={q1: theta1, q2: theta2, q3: theta3})
            R3_6 = R0_3.inv('LU') * ROT_EE

            theta5 = atan2(sqrt(R3_6[0,2]**2 + R3_6[2,2]**2), R3_6[1,2])
            if sin(theta5) > 0:
                theta4 = atan2(R3_6[2,2], -R3_6[0,2])
                theta6 = atan2(-R3_6[1,1], R3_6[1,0])
            else:
                theta4 = atan2(-R3_6[2,2], R3_6[0,2])
                theta6 = atan2(R3_6[1,1], -R3_6[1,0])

            if x >= len(req.poses):
                theta5 = theta5_temp
                theta6 = theta6_temp

            theta5_temp = theta5
            theta6_temp = theta6

            # Populate response for the IK request
            # In the next line replace theta1,theta2...,theta6 by your joint angle variables
    	    joint_trajectory_point.positions = [theta1, theta2, theta3, theta4, theta5, theta6]
    	    joint_trajectory_list.append(joint_trajectory_point)

        rospy.loginfo("length of Joint Trajectory List: %s" % len(joint_trajectory_list))
        return CalculateIKResponse(joint_trajectory_list)

def IK_server():
    # initialize node and declare calculate_ik service
    rospy.init_node('IK_server')
    s = rospy.Service('calculate_ik', CalculateIK, handle_calculate_IK)
    print "Ready to receive an IK request"
    rospy.spin()

if __name__ == "__main__":
    IK_server()
