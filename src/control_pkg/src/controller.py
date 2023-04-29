import rospy
import sys
import numpy as np
from collections import deque

from scipy.spatial.transform import Rotation as R

from trajectories import LinearTrajectory



class Controller(object):
	def __init__(self, limb, kin):


		self._limb = limb
		self._kin = kin

		self.joint_names = ['right_j0', 'right_j1', 'right_j2', 'right_j3',
							'right_j4', 'right_j5', 'right_j6']

		self._kp1 = 0.8
		self._kp2 = 0.8

		kd1 = 0.4
		kd2 = 0.4

		self.Kd = np.diag([kd1,kd1,kd1, kd2,kd2,kd2])

		self.Kp = np.diag([self._kp1, self._kp1, self._kp1, self._kp2, self._kp2, self._kp2])

		self.Kp_admittance = np.diag([0.1,0.1,0.1,0.1,0.1,0.1])
		self.Kd_admittance = np.diag([5,5,5,5,5,5])

		self._LastError = np.zeros(len(self.Kp))
		self._LastTime = 0.0;
		self._ring_buff_capacity = 5
		self._ring_buff = deque([], self._ring_buff_capacity)

		self.vel_sat = np.array([2,2,2,1,1,0.5,0.5])

		dic_pos_init = {self._limb.joint_names()[i]:float(0) for i in range(len(self._limb.joint_names()))}
		ee_init = self._kin.forward_position_kinematics(dic_pos_init)

		self.control_method = 'default'

	def shutdown(self):
		rospy.loginfo("Stopping Controller")

		# Set velocities to zero
		dic_vel = {self._limb.joint_names()[i]:float(0) for i in range(len(self._limb.joint_names()))}
		self._limb.set_joint_velocities(dic_vel)

		rospy.sleep(0.1)

	def execute(self, input, timeout= 100.0, log = False):
		startTime = rospy.Time.now()

		rate = 200
		r = rospy.Rate(rate)

		self.dt = 1.0/rate

		des = input[0]
		des_th = input[1]

		self._LastError = np.zeros(len(self.Kd))
		self._LastTime = 0.0
		self._ring_buff = deque([], self._ring_buff_capacity)

		done = False

		#des should be calculated here
		ee_pos_quat = self._kin.forward_position_kinematics()

		ee_pos = ee_pos_quat[0:3]
		ee_quat = ee_pos_quat[3:7]

		## Currently fixed for 2023/04/28

		R_e = R.from_quat(ee_quat).as_matrix()
		R_d = np.array([[-1, 0, 0],
						[0, 1, 0],
						[0, 0, -1]])

		self.R_d = R_d @ self.rotation_z(des_th)

		start = (ee_pos, R_e)
		final = (des, self.R_d)

		end_time = 15

		self.trajectory = LinearTrajectory(start, final, end_time = end_time)

		while not rospy.is_shutdown():
			t = (rospy.Time.now() - startTime).to_sec()

			if timeout is not None and t >= timeout:
				dic_vel = {name:np.zeros(len(self._limb.joint_names())) for name in self._limb.joint_names()}
				self._limb.set_joint_velocities(dic_vel)
				return False

			# des = np.array([np.pi/8 * np.sin(3*t), np.pi/8 * np.sin(3*t)])
			if self.control_method == 'default':
				u, done = self.step_control_default(t)
			else:
				NotImplementedError('Invalid type of controller')

			if u.shape[1] is not None:
				u_ = np.array([u[0,0], u[0,1], u[0,2], u[0,3], u[0,4], u[0,5], u[0,6]])
				u = u_	

			# print('CI:', u, type(u))
			# print([type(float(i)) for i in u])

			dic_vel = {self._limb.joint_names()[i]:float(u[i]) for i in range(len(self._limb.joint_names()))}
			# print(list(dic_vel.values()))
			self._limb.set_joint_velocities(dic_vel)

			r.sleep()

			if done or t > end_time:
				print(done, t)
				break

		self.shutdown()

		return True

	def step_control_default(self, t):

		# print(des.shape)

		ee_pos_quat = self._kin.forward_position_kinematics()

		ee_pos = ee_pos_quat[0:3]
		ee_quat = ee_pos_quat[3:7]
		R_e = R.from_quat(ee_quat).as_matrix()

		## Currently fixed for 2023/04/28

		(pd, Rd) = self.trajectory.target_config(t)

		vel_des = self.trajectory.target_velocity(t)

		err_vec_trans = ee_pos - pd
		err_vec_rot = self.orientation_error_robosuite(self.R_d, R_e)


		#TODO: Calculate error vector
		error_vector = np.array([err_vec_trans[0], err_vec_trans[1], err_vec_trans[2],
								err_vec_rot[0], err_vec_rot[1], err_vec_rot[2]])

		#-----derivative term
		dt = t - self._LastTime
		print(ee_pos)
		curr_derivative = (error_vector - self._LastError) / dt
		# print('curr_derivative', curr_derivative)
		self._ring_buff.append(curr_derivative)
		# print('ring_buff', self._ring_buff)

		ed = np.mean(self._ring_buff, axis = 0 )

		self._LastError = error_vector
		self._LastTime = t

		J = self._kin.jacobian()
		dq_dict = self._limb.joint_velocities()

		dq = np.zeros((7,))
		for i, joint in enumerate(dq_dict.keys()):
			dq[i] = dq_dict[joint]
		
		dq = dq.reshape((-1,1))

		J_pinv_our = np.linalg.pinv(J.T @ J) @ J.T

		# J_pinv = self._kin.jacobian_pseudo_inverse()
		# print(ed, ed.shape)

		u = J_pinv_our @ (- self.Kp @ error_vector.reshape((-1,1)) - self.Kd @ ed.reshape((-1,1)) + vel_des.reshape((-1,1)))

		# u = np.clip(u.reshape((-1,)), -self.vel_sat, self.vel_sat)

		u = u.reshape((-1,))

		u = u.astype(np.float64)

		done = False
		# if np.linalg.norm(err_vec_trans) < 0.01 and np.linalg.norm(err_vec_rot) < 0.03:
			# pass
			# done = True

		return u, done

	def vee_map(self, R):
		return np.array([-R[1,2], R[0,2], -R[0,1]])

	def rotation_z(self,th):
		s_t = np.sin(th)
		c_t = np.cos(th)

		Rz = np.array([[c_t, -s_t, 0],
					   [s_t, c_t, 0],
					   [0, 0, 1]])
		return Rz

	def orientation_error_robosuite(self, desired, current):
		rc1 = current[0:3, 0]
		rc2 = current[0:3, 1]
		rc3 = current[0:3, 2]
		rd1 = desired[0:3, 0]
		rd2 = desired[0:3, 1]
		rd3 = desired[0:3, 2]

		error = -0.5 * (np.cross(rc1, rd1) + np.cross(rc2, rd2) + np.cross(rc3, rd3))

		return error

	def orientation_error_ours(self, R_d, R_e):
		R_ed = R_d.T @ R_e
		error = 0.5 * self.vee_map(R_ed - R_ed.T)

		return error





