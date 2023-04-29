#!/usr/bin/env/python

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import argparse

from utils import log_map, hat_map
import scipy as sc


"""
Set of classes for defining SE(3) trajectories for the end effector of a robot 
manipulator
"""

class Trajectory:

    def __init__(self, total_time):
        self.total_time = total_time

    def target_pose(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        pass

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.
        
        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """
        pass

class LinearTrajectory(Trajectory):

    def __init__(self, start_point, final_point, start_time = 0, end_time = 10):
        """
        Remember to call the constructor of Trajectory

        Parameters
        ----------
        start_point = (p_s, R_s) should be given in the array
        final_point = (p_f, R_f)
        """
        #super().__init__()
        Trajectory.__init__(self, end_time - start_time)
        self.start_pos, self.start_rot = start_point
        self.final_pos, self.final_rot = final_point
        self.start_time = start_time
        self.end_time = end_time

        a,b,c,d = self.calculate_param_pos()
        self.a_p = a
        self.b_p = b
        self.c_p = c
        self.d_p = d

        self.a_r, self.b_r, self.c_r, self.d_r = self.calculate_param_rot()

    def calculate_param_pos(self):
        A = np.array([[1, self.start_time, self.start_time**2, self.start_time**3], 
                      [1, self.end_time, self.end_time**2, self.end_time**3],
                      [0, 1, 2*self.start_time, 3*self.start_time**2],
                      [0, 1, 2*self.end_time, 3*self.end_time**2]])
        b_x = np.array([self.start_pos[0], self.final_pos[0], 0, 0])
        b_y = np.array([self.start_pos[1], self.final_pos[1], 0, 0])
        b_z = np.array([self.start_pos[2], self.final_pos[2], 0, 0])

        param_x = np.linalg.inv(A) @ b_x.reshape((-1,1))
        param_y = np.linalg.inv(A) @ b_y.reshape((-1,1))
        param_z = np.linalg.inv(A) @ b_z.reshape((-1,1))

        a = np.array([param_x[0,0], param_y[0,0], param_z[0,0]])
        b = np.array([param_x[1,0], param_y[1,0], param_z[1,0]])
        c = np.array([param_x[2,0], param_y[2,0], param_z[2,0]])
        d = np.array([param_x[3,0], param_y[3,0], param_z[3,0]])
        
        return a,b,c,d

    def calculate_param_rot(self):
        A = np.array([[1, self.start_time, self.start_time**2, self.start_time**3], 
                      [1, self.end_time, self.end_time**2, self.end_time**3],
                      [0, 1, 2*self.start_time, 3*self.start_time**2],
                      [0, 1, 2*self.end_time, 3*self.end_time**2]])
        
        start_log = log_map(np.eye(3))
        final_log = log_map(self.start_rot.T @ self.final_rot)

        b_0 = np.array([start_log[0], final_log[0], 0, 0])
        b_1 = np.array([start_log[1], final_log[1], 0, 0])
        b_2 = np.array([start_log[2], final_log[2], 0, 0])

        param_0 = np.linalg.inv(A) @ b_0.reshape((-1,1))
        param_1 = np.linalg.inv(A) @ b_1.reshape((-1,1))
        param_2 = np.linalg.inv(A) @ b_2.reshape((-1,1))

        a = np.array([param_0[0,0], param_1[0,0], param_2[0,0]])
        b = np.array([param_0[1,0], param_1[1,0], param_2[1,0]])
        c = np.array([param_0[2,0], param_1[2,0], param_2[2,0]])
        d = np.array([param_0[3,0], param_1[3,0], param_2[3,0]])

        return a,b,c,d

    def target_config(self, time):
        """
        Returns where the arm end effector should be at time t, in the form of a 
        7D vector [x, y, z, qx, qy, qz, qw]. i.e. the first three entries are 
        the desired end-effector position, and the last four entries are the 
        desired end-effector orientation as a quaternion, all written in the 
        world frame.

        Hint: The end-effector pose with the gripper pointing down corresponds 
        to the quaternion [0, 1, 0, 0]. 

        Parameters
        ----------
        time : float        
    
        Returns
        -------
        7x' :obj:`numpy.ndarray`
            desired configuration in workspace coordinates of the end effector
        """
        xd = self.a_p + self.b_p * time + self.c_p * time**2 + self.d_p * time**3
        wt = self.a_r + self.b_r * time + self.c_r * time**2 + self.d_r * time**3

        Rt = sc.linalg.expm(hat_map(wt))

        Rd = self.start_rot @ Rt

        ### Should return Rd instead of qd

        target_config = (xd, Rd)

        return target_config

    def target_velocity(self, time):
        """
        Returns the end effector's desired body-frame velocity at time t as a 6D
        twist. Note that this needs to be a rigid-body velocity, i.e. a member 
        of se(3) expressed as a 6D vector.

        Parameters
        ----------
        time : float

        Returns
        -------
        6x' :obj:`numpy.ndarray`
            desired body-frame velocity of the end effector
        """

        vd = self.b_p + 2*self.c_p*time + 3*self.d_p*time**2
        #print(vd)
        wd = np.array([0,0,0])
        return np.hstack((vd,wd))
