#!/usr/bin/env python

import sys
import numpy as np
import itertools
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from intera_interface import Limb
from sawyer_pykdl import sawyer_kinematics

from intera_interface import gripper as robot_gripper
# intera_core_msgs/EndpointState

import traceback

try:
    import tf
    import tf2_ros
    import rospy
    import baxter_interface
    import intera_interface
    from geometry_msgs.msg import PoseStamped
    from moveit_msgs.msg import RobotTrajectory
    from cv_pkg.msg import grasp_msg
    
    from intera_core_msgs.msg import EndpointState
except:
    pass

try:
	from controller import Controller
except ImportError:
	print('controller not imported')
	pass

NUM_JOINTS = 7
data = np.array([0,0,0])

def main():

	des = np.array([0.6, 0.2, 0.15])
	des_th = 0
	input_ = (des, des_th)

	# while not rospy.is_shutdown():
		
	

	controller = Controller(limb = Limb(),kin = sawyer_kinematics('right'), control_method = 'default')

	while not rospy.is_shutdown():
		try:
			input('Make sure that your hand is on the E-stop and press <Enter>')
			# rospy.Subscriber("/robot/limb/right/endpoint_state", EndpointState, callback)
			if not controller.execute(input_):
				raise Exception("Execution failed")
		except Exception as e:
			print(e)
			traceback.print_exc()
		else:
			break

def callback(message):
	ee_force = message.wrench.force
	x = ee_force.x
	y = ee_force.y
	z = ee_force.z
	print('forces are:', x,y,z)

if __name__ == '__main__':
	rospy.init_node('test_node')
	main()
