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
     
            # Calculate joint angles using Geometric IK method
            lx = cos(roll)*cos(pitch)
            ly = sin(roll)*cos(pitch)
            lz = -sin(pitch)

            gripper_length = 0.303 #d6 +d7

            wx = px-(gripper_length)*lx
            wy = py-(gripper_length)*ly
            wz = pz-(gripper_length)*lz
		
	    q1 = atan2(wy,wx)

            #q2 position
            x_2 = 0.35 * cos(q1)
            y_2 = 0.35 * sin(q1)
            z_2 = 0.75
            a2 = 1.25
            d3 = 1.5

            #distance q2-q4
            dist1 = sqrt((wx-x_2)**2.0 + (wy-y_2)**2.0 + (wz-z_2)**2.0)

            q3 = acos((a2**2.0 + d3**2.0 - dist1**2.0)/(2.0*a2*d3))
            q3 = (q3 - pi/2.0) * -1.0 #conversion to robot joint angle

            q2= atan2(wz-z_2,sqrt((wx-x_2)**2 + (wy-y_2)**2)) + acos((a2**2.0 - d3**2.0 + dist1**2.0)/(2.0*a2*dist1)) 
            q2 = (q2 - pi/2.0) * -1.0 #conversion to robot joint angle
            
            #Roll and pitch in wrist coordinates space
            roll2 = roll - q1
            pitch2 = pitch - (q2 + q3)
            #End efector projection in axe
            dgry = 0.303*cos(pitch2)*sin(roll2)
            dgrz = 0.303*sin(pitch2)

            #wrist angles
            q4 = atan2(dgry, dgrz)
            q5 = atan2(sqrt(dgry**2+dgrz**2),cos(pitch2)*cos(roll2))
            #clip q5 to avoid forbbiden values
            if q5 > 2.18:
                q5 = 2.18
            if q5 < -2.18:
                q5 = -2.18

            if pitch>-pi/2 and pitch<pi/2:
                q6 = yaw - q4
            else:
                q6 = pi - (yaw - q4)
		
	    # Populate response for the IK request
            joint_trajectory_point.positions = [q1, q2, q3, q4, q5, q6]
            #rospy.loginfo("Angles: %s" % joint_trajectory_point.positions)
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
