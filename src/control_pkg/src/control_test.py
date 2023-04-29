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
	# -0.03 should be our z-desired point
	ball_pos = np.array([0.587, 0.199, -0.03])#### should be obtained

	right_gripper = robot_gripper.Gripper('right_gripper')	
	rospy.Subscriber("grasp_info", grasp_msg, callback)

	inc = np.array([data[0], -data[1], 0])

	desired_pos = ball_pos + inc
	theta = data[2]
	input_ = (desired_pos,theta)

	right_gripper.open()
	print(input_)

	controller = Controller(limb = Limb(),kin = sawyer_kinematics('right'))

	
	right_gripper.calibrate()
	right_gripper.open()
	rospy.sleep(3.0)

	while not rospy.is_shutdown():
		try:
			input('Make sure that your hand is on the E-stop and press <Enter>')
			if not controller.execute(input_):
				raise Exception("Execution failed")
		except Exception as e:
			print(e)
			traceback.print_exc()
		else:
			break

	right_gripper.open()
	rospy.sleep(1.0)
	print('Opening')
	right_gripper.close()
	rospy.sleep(2.0)

	# Open the right gripper
	# print('Opening...')
	# right_gripper.open()
	rospy.sleep(1.0)
	print('Done!')

	input_ = (np.array([0.6, 0, 0.5]), 0)

	while not rospy.is_shutdown():
		try:
			input('Make sure that your hand is on the E-stop and press <Enter>')
			if not controller.execute(input_):
				raise Exception("Execution failed")
		except Exception as e:
			print(e)
			traceback.print_exc()
		else:
			break

def callback(message):
	print(message)
	x = message.grasp_x_cm
	y = message.grasp_y_cm
	theta = message.grasp_theta
	global data
	data = np.array([x,y,theta])
	

if __name__ == '__main__':
	rospy.init_node('test_node')
	main()
