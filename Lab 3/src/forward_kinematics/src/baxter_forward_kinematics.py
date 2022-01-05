#!/usr/bin/env python

import numpy as np
import scipy as sp
import kin_func_skeleton as kfs

def baxter_forward_kinematics_from_angles(joint_angles):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    angles in radians.

    Parameters
    ----------
    joint_angles ((7x) np.ndarray): 7 joint angles (s0, s1, e0, e1, w0, w1, w2)

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """

    qs = np.ndarray((3,8)) # points on each joint axis in the zero configuration
    ws = np.ndarray((3,7)) # axis vector of each joint axis
    
    # Assign the q values
    qs[0:3,0] = [0.0635, 0.2598, 0.1188]
    qs[0:3,1] = [0.1106, 0.3116, 0.3885]
    qs[0:3,2] = [0.1827, 0.3838, 0.3881]
    qs[0:3,3] = [0.3682, 0.5684, 0.3181]
    qs[0:3,4] = [0.4417, 0.6420, 0.3177]
    qs[0:3,5] = [0.6332, 0.8337, 0.3067]
    qs[0:3,6] = [0.7152, 0.9158, 0.3063]
    qs[0:3,7] = [0.7957, 0.9965, 0.3058]

    # Assign the w values
    ws[0:3,0] = [-0.0059,  0.0113,  0.9999]
    ws[0:3,1] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,2] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,3] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,4] = [ 0.7065,  0.7077, -0.0038]
    ws[0:3,5] = [-0.7077,  0.7065, -0.0122]
    ws[0:3,6] = [ 0.7065,  0.7077, -0.0038]

    R = np.array([[0.0076, 0.0001, -1.0000],
                    [-0.7040, 0.7102, -0.0053],
                    [0.7102, 0.7040, 0.0055]]).T # rotation matrix of zero config

    # YOUR CODE HERE (Task 1)
    twist = np.ndarray((6,7))
    twist[0:3,0] = -np.cross([ws[0:3,0]],qs[0:3,0])
    twist[3:6,0] = ws[0:3,0]
    twist[0:3,1] = -np.cross([ws[0:3,1]],qs[0:3,1])
    twist[3:6,1] = ws[0:3,1]
    twist[0:3,2] = -np.cross([ws[0:3,2]],qs[0:3,2])
    twist[3:6,2] = ws[0:3,2]
    twist[0:3,3] = -np.cross([ws[0:3,3]],qs[0:3,3])
    twist[3:6,3] = ws[0:3,3]
    twist[0:3,4] = -np.cross([ws[0:3,4]],qs[0:3,4])
    twist[3:6,4] = ws[0:3,4]
    twist[0:3,5] = -np.cross([ws[0:3,5]],qs[0:3,5])
    twist[3:6,5] = ws[0:3,5]
    twist[0:3,6] = -np.cross([ws[0:3,6]],qs[0:3,6])
    twist[3:6,6] = ws[0:3,6]

    gst0 = np.array([[R[0,0],R[0,1],R[0,2],qs[0,7]],[R[1,0],R[1,1],R[1,2],qs[1,7]],[R[2,0],R[2,1],R[2,2],qs[2,7]],[0,0,0,1]])
    g = np.matmul(kfs.prod_exp(twist,joint_angles),gst0)

    return g



def baxter_forward_kinematics_from_joint_state(joint_state):
    """
    Computes the orientation of the Baxter's left end-effector given the joint
    state.

    Parameters
    ----------
    joint_state (sensor_msgs.JointState): JointState of Baxter robot

    Returns
    -------
    (4x4) np.ndarray: homogenous transformation matrix
    """
    
    angles = np.zeros(7)

    # YOUR CODE HERE (Task 2)

    # s0 = 4, s1 = 5, e0 = 2, e1 = 3, w0 = 6, w1 = 7, w2 = 8
    angles[0] = joint_state.position[4]
    angles[1] = joint_state.position[5]
    angles[2] = joint_state.position[2]
    angles[3] = joint_state.position[3]
    angles[4] = joint_state.position[6]
    angles[5] = joint_state.position[7]
    angles[6] = joint_state.position[8]

    return baxter_forward_kinematics_from_angles(angles)

    print(baxter_forward_kinematics_from_angles(angles))