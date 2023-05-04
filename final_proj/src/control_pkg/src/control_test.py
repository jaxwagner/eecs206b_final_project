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

class ControllerNode:
	def __init__(self):
		self.data = np.array([0,0,0])
		self.camera_center = np.array([0.545, 0.086, -0.015])#### z is bottom point
		# self.camera_center = np.array([0.525, 0.072, -0.005])#### should be obtained

		self.dict_vel = {}

		#print('initialized')

	def main(self):
		# print('i am here1')
		right_gripper = robot_gripper.Gripper('right_gripper')
		# print('i am here2')

		rospy.Subscriber("grasp_info", grasp_msg, self.callback)
		dict_val = {}
		# self.data[0] = round(self.data[0], 3)
		# self.data[1] = round(self.data[1], 3)

		self.arr = []

		for i in range(100):
			rospy.sleep(0.1)
			print(self.data)
			# print('i am in the loop')
			# tup_list = tuple([self.data[0], self.data[1]])
			# if tup_list in dict_val:
			# 	dict_val[tup_list]+=1
			# else:
			# 	dict_val[tup_list]=1

		# a = np.asarray(self.arr)
		# N,d = a.shape
		# sum1 = 0
		# sum2 = 0
		# sum3 = 0
		# for i in range(N):
		# 	sum1 += a[i,0]
		# 	sum2 += a[i,1]
		# 	sum3 += a[i,2]

		# print(sum1, sum2, sum3)

		# self.data[0] = sum1 / N
		# self.data[1] = sum2 / N
		# self.data[2] = sum3 / N

		# print(self.data)


		# print(dict_val)
		inc = np.array([self.data[0], -self.data[1], 0])

		desired_pos = self.camera_center + inc
		theta = self.data[2]
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

		input_ = (np.array([0.6, 0.2, 0.5]), 0)

		while not rospy.is_shutdown():
			try:
				# input('Make sure that your hand is on the E-stop and press <Enter>')
				if not controller.execute(input_):
					raise Exception("Execution failed")
			except Exception as e:
				print(e)
				traceback.print_exc()
			else:
				break

		right_gripper.open()

	def callback(self, message):
		# print(message)
		x = message.grasp_x_cm * 0.01 # in meter
		y = message.grasp_y_cm * 0.01 # in meter
		theta = message.grasp_theta
		# x = round(x,2)
		# y = round(y,2)
		# theta = round(theta,2)
		# print("Jake print", x, y, theta)
		

		self.data = np.array([x,y,theta])

		# self.arr.append(np.array([x,y,theta]))


# def main():
# 	# -0.03 should be our z-desired point
# 	camera_center = np.array([0.525, 0.072, -0.015])#### should be obtained

# 	# k = 0
# 	# for k in range(100):
		
# 		# k = k+1
# 	right_gripper = robot_gripper.Gripper('right_gripper')	
# 	rospy.Subscriber("grasp_info", grasp_msg, callback)

# 	inc = np.array([data[0], -data[1], 0])

# 	desired_pos = camera_center + inc
# 	theta = data[2]
# 	input_ = (desired_pos,theta)

# 	right_gripper.open()
# 	print(input_)

# 	controller = Controller(limb = Limb(),kin = sawyer_kinematics('right'))

	
# 	right_gripper.calibrate()
# 	right_gripper.open()
# 	rospy.sleep(3.0)

# 	while not rospy.is_shutdown():
# 		try:
# 			input('Make sure that your hand is on the E-stop and press <Enter>')
# 			if not controller.execute(input_):
# 				raise Exception("Execution failed")
# 		except Exception as e:
# 			print(e)
# 			traceback.print_exc()
# 		else:
# 			break

# 	right_gripper.open()
# 	rospy.sleep(1.0)
# 	print('Opening')
# 	right_gripper.close()
# 	rospy.sleep(2.0)

# 	# Open the right gripper
# 	# print('Opening...')
# 	# right_gripper.open()
# 	rospy.sleep(1.0)
# 	print('Done!')

# 	input_ = (np.array([0.6, 0.2, 0.5]), 0)

# 	while not rospy.is_shutdown():
# 		try:
# 			input('Make sure that your hand is on the E-stop and press <Enter>')
# 			if not controller.execute(input_):
# 				raise Exception("Execution failed")
# 		except Exception as e:
# 			print(e)
# 			traceback.print_exc()
# 		else:
# 			break

# def callback(message):
# 	# print(message)
# 	x = message.grasp_x_cm * 0.01 # in meter
# 	y = message.grasp_y_cm * 0.01 # in meter
# 	theta = message.grasp_theta
# 	global data
# 	data = np.array([x,y,theta])
	

if __name__ == '__main__':
	rospy.init_node('test_node')
	# print('i am here')
	control_node = ControllerNode()
	control_node.main()
