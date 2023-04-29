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
except:
    pass

try:
	from controller import Controller
except ImportError:
	print('controller not imported')
	pass

NUM_JOINTS = 7

def main():
	# -0.03 should be our z-desired point
	des = np.array([0.6, -0.4, 0.5])

	controller = Controller(limb = Limb(),kin = sawyer_kinematics('right'))

	right_gripper = robot_gripper.Gripper('right_gripper')
	# right_gripper.calibrate()
	right_gripper.open()
	rospy.sleep(2.0)
	print('Closing...')
	# right_gripper.close()
	rospy.sleep(1.0)

	# Open the right gripper
	# print('Opening...')
	# right_gripper.open()
	rospy.sleep(1.0)
	print('Done!')

	# while not rospy.is_shutdown():
	# 	try:
	# 		input('Make sure that your hand is on the E-stop and press <Enter>')
	# 		if not controller.execute(des):
	# 			raise Exception("Execution failed")
	# 	except Exception as e:
	# 		print(e)
	# 		traceback.print_exc()
	# 	else:
	# 		break


if __name__ == '__main__':
	rospy.init_node('test_node')
	main()
