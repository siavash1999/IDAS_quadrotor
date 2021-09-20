#!/usr/bin/env python3
#--------------------------------------------------------------------------------------------------
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from quadrotor.controller import Correction
#--------------------------------------------------------------------------------------------------
def cruise_control(msg):

	#Defining a Float64MultiArray Object for publishing Motor Velocities
	motor_vels = Float64MultiArray()
	K_cruise = rospy.get_param('/quadrotor/controller/k_cruise')
	Ct = rospy.get_param('/quadrotor/aerodynamic/ct')
	Cm = rospy.get_param('/quadrotor/aerodynamic/cm')
	Cd = rospy.get_param('/quadrotor/aerodynamic/cd')
	d = rospy.get_param('/quadrotor/geometry/d')
	m = rospy.get_param('/quadrotor/inertia/mass')
	g = 9.84

	FtoS = 0.25*np.array([[1, -np.sqrt(2), -np.sqrt(2),  1],\
	                      [1,  np.sqrt(2), -np.sqrt(2), -1],\
	                      [1,  np.sqrt(2),  np.sqrt(2),  1],\
	                      [1, -np.sqrt(2),  np.sqrt(2), -1]])
	u = 1
	theta = np.arcsin(-Cd*(u**2)/(m*g))
	w = u*np.tan(theta)

	setpoint = np.array([0, theta, 0, 0, 0, 0, u, 0, w])
	current_state = np.array(msg.data)
	error_signal = Correction(current_state - setpoint)
	input_signal_temp = np.dot(K_cruise[0],error_signal)
	input_signal = [0, 0, 0, 0]
	input_signal[0] = (input_signal_temp[0] + m*g*np.cos(theta)+Cd*(w**2))/Ct
	input_signal[1] = input_signal_temp[1]/(d*Ct)
	input_signal[2] = input_signal_temp[2]/(d*Ct)
	input_signal[3] = input_signal_temp[3]/Cm

	#transfering input signal to get motors rotation speed:
	vels_temp = np.dot(FtoS, input_signal)
	vels = np.sign(vels_temp)*np.sqrt(np.abs(vels_temp))
	vels[1] = -vels[1]
	vels[3] = -vels[3]
	#setting the floor and ceiling for motor speed so it won't go crazy:
	max_vel = 1500
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

	VelPub.publish(motor_vels)
	uPub.publish(current_state[6])
	wPub.publish(current_state[8])
	thetaPub.publish(current_state[1])

#--------------------------------------------------------------------------------------------------
#Initiate the node that will control the gazebo model
rospy.init_node("cruise_control")

#initialte publisher velPub that will publish the velocities of individual BLDC motors
VelPub = rospy.Publisher('/quadrotor/joint_motor_controller/command', Float64MultiArray, queue_size = 50)
uPub = rospy.Publisher('/u_cruise', Float64, queue_size = 50)
wPub = rospy.Publisher('/w_cruise', Float64, queue_size = 50)
thetaPub = rospy.Publisher('/theta_cruise', Float64, queue_size = 50)
#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
CruiseSub = rospy.Subscriber('quadrotor/cruise_states', Float64MultiArray, cruise_control)

rospy.spin()
