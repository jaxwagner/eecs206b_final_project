#!/usr/bin/env python

import sys
import numpy as np
import itertools
import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt

from intera_interface import Limb
from sawyer_pykdl import sawyer_kinematics
from collections import defaultdict

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

	def main(self):
		right_gripper = robot_gripper.Gripper('right_gripper')

		rospy.Subscriber("grasp_info", grasp_msg, self.callback)

		points = []
		
		for i in range(250):
			rospy.sleep(0.05)
			points.append(tuple([round(self.data[0],3), round(self.data[1],3), round(self.data[2], 1)]))
		
		grid = defaultdict(list)
		for curr_point in points:
			distance_dict = {}
			for other_point in points:
				distance = round(np.sqrt((curr_point[0] - other_point[0])**2 + (curr_point[1] - other_point[1])**2), 5)
				distance_dict[other_point] = distance
			#Get distance of all the points from this curr_point and sort it
			output = sorted(distance_dict.items(), key=lambda x: x[1])
			#print(output)
			for point_distance in output:
				point = point_distance[0]
				distance = point_distance[1]
				#If distance is less than some threshold, consider it as if they are in the same cluster.
				if distance < 0.01:
					grid[curr_point].append(point)
				else:
					break
		#print(grid)
		most_points_cluster = sorted(grid.items(), key=lambda x: len(x[1]), reverse = True)
		x = []
		y = []
		desired_point = most_points_cluster[0]
		theta_map = {}
		#print(desired_point)

		for other_point in desired_point[1]:
			x.append(other_point[0])
			y.append(other_point[1])
			if other_point[2] in theta_map:
				theta_map[other_point[2]] +=1
			else:
				theta_map[other_point[2]] = 1
		sorted_theta_map = sorted(theta_map.items(), key=lambda x: x[1], reverse = True)[0]
		print(sorted_theta_map)
		theta = sorted_theta_map[0]
		plt.scatter(x, y)
		plt.plot(desired_point[0][0], desired_point[0][1],'ro')
		plt.show() 
	

		#These are the points within the center point that has the most number of points in the cluster. So blue points are neighbor points that are within the cluster
		
		#Red point is a cluster center. For now it's just a point, but
		#TODO: Make this a centroid of the cluster, not just the point with most number of clustered points
		
		print(desired_point[0][0], desired_point[0][1], theta)
		
		
		inc = np.array([desired_point[0][0], -desired_point[0][1], 0])

		desired_pos = self.camera_center + inc
		#theta = self.data[2]
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
	control_node = ControllerNode()
	control_node.main()
