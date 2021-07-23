from control import lqr
import numpy as np
from time import time
import rospy

#Declare global variables and getting them from parameter server
global d, m, Ix, Iy, Iz, Ct, Cm, Cd, g
d = rospy.get_param('/quadrotor/geometry/d')
m = rospy.get_param('/quadrotor/inertia/mass')
Ix = rospy.get_param('/quadrotor/inertia/Ixx')
Iy = rospy.get_param('/quadrotor/inertia/Iyy')
Iz = rospy.get_param('/quadrotor/inertia/Izz')
Ct = rospy.get_param('/quadrotor/aerodynamic/ct')
Cm = rospy.get_param('/quadrotor/aerodynamic/cm')
Cd = rospy.get_param('/quadrotor/aerodynamic/cd')
g = 9.84

#--------------------------------------------------------------------------------------------------
def Correction2D(K):
	for i in range(len(K)):
		for j in range(len(K[0])):
			if abs(K[i][j]) < 1e-4:
				K[i][j] = 0
	return K
#--------------------------------------------------------------------------------------------------
def Correction(K):
	for i in range(len(K)):
		if abs(K[i]) <1e-4:
			K[i] = 0
	return K
#-------------------------
def Hover(roll, pitch, yaw, p, q, r, u, v, w, x, y, z, motor_vels):
	
	FtoS = 0.25*np.array([[1,  np.sqrt(2),  np.sqrt(2),  1],\
	                      [1, -np.sqrt(2),  np.sqrt(2), -1],\
	                      [1, -np.sqrt(2), -np.sqrt(2),  1],\
	                      [1,  np.sqrt(2), -np.sqrt(2), -1]])
	
	setpoint = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5])
	current_state = np.array([roll, pitch, yaw, p, q, r, u, v, w, x, y, z])
	error_signal = Correction(current_state - setpoint)
	#----------------------------------------------------------------------------------------------
	#Define State and Input Matrices:
	A = [[0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],\
             [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0,-g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [g, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0]]
	B = [[0   , 0   , 0   , 0   ],\
	     [0   , 0   , 0   , 0   ],\
	     [0   , 0   , 0   , 0   ],\
	     [0   , 1/Ix, 0   , 0   ],\
	     [0   , 0   , 1/Iy, 0   ],\
	     [0   , 0   , 0   , 1/Iz],\
	     [0   , 0   , 0   , 0   ],\
	     [0   , 0   , 0   , 0   ],\
	     [-1/m, 0   , 0   , 0   ],\
	     [0   , 0   , 0   , 0   ],\
	     [0   , 0   , 0   , 0   ],\
	     [0   , 0   , 0   , 0   ]]
	  
	#Defining weight matrices Q and R for state and input matrices respectively:
	Q = [[5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 5, 0, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 1, 0, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 5, 0],\
	     [0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 10]]
	R = [[1, 0,  0,  0],\
	     [ 0,1,  0,  0],\
	     [ 0, 0, 1,  0],\
	     [ 0, 0,  0, 1]]
	
	#Calculating Matrix K gain with LQR method:
	K, S, E = lqr(A, B, Q, R)
	K = np.array(Correction2D(K))
	
	#Computing Input signals based on Moments and thrust force:
	input_signal_temp = np.dot(K,error_signal)
	input_signal = [0, 0, 0, 0]
	input_signal[0] = (input_signal_temp[0] + m*g)/Ct
	input_signal[1] = input_signal_temp[1]/(d*Ct)
	input_signal[2] = input_signal_temp[2]/(d*Ct)
	input_signal[3] = input_signal_temp[3]/Cm
	
	#transfering input signal to get motors rotation speed:
	vels_temp = np.dot(FtoS, input_signal)
	vels = np.sign(vels_temp)*np.sqrt(np.abs(vels_temp))
	vels[1] = -vels[1]
	vels[3] = -vels[3]
	#setting the floor and ceiling for motor speed so it won't go crazy:
	max_vel = 900
	if vels[0] > max_vel:
		vels[0] = max_vel
	if vels[0] < -max_vel:
		vels[0] = -max_vel
	
	if vels[1] < -max_vel:
		vels[1] = -max_vel
	if vels[1] > max_vel:
		vels[1] = max_vel
	
	if vels[2] > max_vel:
		vels[2] = max_vel
	if vels[2] < -max_vel:
		vels[2] = -max_vel
	
	if vels[3] < -max_vel:
		vels[3] = -max_vel
	if vels[3] > max_vel:
		vels[3] = max_vel
	
	motor_vels.data = vels.tolist()
	
	#Return these variables back to the control file.
	return [motor_vels]
