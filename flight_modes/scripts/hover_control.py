#!/usr/bin/env python3
#--------------------------------------------------------------------------------------------------
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32, Float64
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
from flight_modes.qrotation import qv_mult
from flight_modes.hover import Hover

#--------------------------------------------------------------------------------------------------
def Hover_quad(msg, arg):
		
	#Defining a Float64MultiArray Object for publishing Motor Velocities
	motor_vels = Float64MultiArray()
	
	#The ModelStates message from gazebo msgs contains states (poses and velocities) of all
	#objects in the simulation (Ground, quadrotor, etc...) in "pose" and "twist" message forms
	#so in order to get access to quadrotor states, we have to know the index in which desired
	#data is stored.
	index = msg.name.index("quadrotor")
	orientObj = msg.pose[index].orientation
	poseObj = msg.pose[index].position
	velObj = msg.twist[index].linear
	avelObj = msg.twist[index].angular
	orientList = [orientObj.x, orientObj.y, orientObj.z, orientObj.w]
	[x, y, z] = [poseObj.x, poseObj.y, poseObj.z]
	[p, q, r] = qv_mult(orientList, [avelObj.x, avelObj.y, avelObj.z])
	[u, v, w] = qv_mult(orientList, [velObj.x, velObj.y, velObj.z])

	altit.publish(z)
	#Convert the quaternion data to roll, pitch, yaw data
	(roll, pitch, yaw) = euler_from_quaternion(orientList)
	
	#Send states to LQR function to recieve desired motor velocities
	[motor_vels] = Hover(roll, pitch, yaw, p, q, r, u, v, w, x, y, z, motor_vels)
	
	#The object args contains the tuple of objects (velPub, err_rollPub, err_pitchPub, err_yawPub). publish the information to namespace.
	arg.publish(motor_vels)

#--------------------------------------------------------------------------------------------------
#Initiate the node that will control the gazebo model
rospy.init_node("hover_control")

#initialte publisher velPub that will publish the velocities of individual BLDC motors
velPub = rospy.Publisher('/quadrotor/joint_motor_controller/command', Float64MultiArray, queue_size = 50)
altit = rospy.Publisher('/altitude', Float64, queue_size = 1)
#Subscribe to /gazebo/model_states to obtain the pose in quaternion form
#Upon receiveing the messages, the objects msg, velPub, err_rollPub, err_pitchPub and err_yawPub are sent to "Control_quad" function.
PoseSub = rospy.Subscriber('/gazebo/model_states_throttle', ModelStates, Hover_quad, (velPub))

rospy.spin()
