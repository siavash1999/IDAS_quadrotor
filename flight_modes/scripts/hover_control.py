#!/usr/bin/env python3
#--------------------------------------------------------------------------------------------------
import rospy
import numpy as np
from std_msgs.msg import Float64MultiArray
from quadrotor.controller import Correction
#--------------------------------------------------------------------------------------------------
def hover_control(msg):

	#Defining a Float64MultiArray Object for publishing Motor Velocities
	motor_vels = Float64MultiArray()
	err_sig = Float64MultiArray()
	effort = Float64MultiArray()
	K_hover = rospy.get_param('/quadrotor/controller/k_hover')
	Ct = rospy.get_param('/quadrotor/aerodynamic/ct')
	Cm = rospy.get_param('/quadrotor/aerodynamic/cm')
	d = rospy.get_param('/quadrotor/geometry/d')
	m = rospy.get_param('/quadrotor/inertia/mass')
	g = 9.81

	FtoS = 0.25*np.array([[1, -np.sqrt(2), -np.sqrt(2),  1],\
	                      [1, -np.sqrt(2),  np.sqrt(2), -1],\
	                      [1,  np.sqrt(2),  np.sqrt(2),  1],\
	                      [1,  np.sqrt(2), -np.sqrt(2), -1]])

	setpoint = np.array([0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 1])
	current_state = np.array(msg.data)
	error_signal = Correction(current_state - setpoint)
	input_signal_temp = np.dot(K_hover,error_signal)
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
	err_sig.data = error_signal.tolist()
	effort.data = input_signal_temp.tolist()
	effort.data[0] = effort.data[0] + m*g

	VelPub.publish(motor_vels)
	errPub.publish(err_sig)
	effortPub.publish(effort)

#--------------------------------------------------------------------------------------------------
#Initiate the node that will control the gazebo model
rospy.init_node("hover_control")

#initialte publisher velPub that will publish the velocities of individual BLDC motors
VelPub = rospy.Publisher('/quadrotor/motor_controller/command', Float64MultiArray, queue_size = 50)
errPub = rospy.Publisher('/err_signal', Float64MultiArray, queue_size = 1)
effortPub = rospy.Publisher('/effort_signal', Float64MultiArray, queue_size = 1)
#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
#Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "Control_quad" function.
HoverSub = rospy.Subscriber('quadrotor/hover_states', Float64MultiArray, hover_control)

rospy.spin()
