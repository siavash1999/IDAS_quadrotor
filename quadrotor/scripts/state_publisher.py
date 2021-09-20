#!/usr/bin/env python3
# ------------------------------------------------------------------------------
import rospy
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32, Float64
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
from quadrotor.qrotation import qv_mult
import quadrotor.controller

# ------------------------------------------------------------------------------
def spliter(msg):
	desired_altitude = 1
	index = msg.name.index("quadrotor")
	orientObj = msg.pose[index].orientation
	poseObj = msg.pose[index].position
	velObj = msg.twist[index].linear
	avelObj = msg.twist[index].angular
	orientList = [orientObj.x, orientObj.y, orientObj.z, orientObj.w]
	[x, y, z] = [poseObj.x, poseObj.y, poseObj.z]
	[p, q, r] = qv_mult(orientList, [avelObj.x, avelObj.y, avelObj.z])
	[u, v, w] = qv_mult(orientList, [velObj.x, velObj.y, velObj.z])
	# Convert the quaternion data to roll, pitch, yaw data
	(yaw, pitch, roll) = euler_from_quaternion(orientList, 'szyx')

	#
	AltitudePub.publish(z)
	AltitudeSetpoint.publish(desired_altitude)
	return None

# ------------------------------------------------------------------------------
# State publisher node initialization
rospy.init_node('state_publisher')

# Initializing Publishers
AltitudePub = rospy.Publisher('/quadrotor/altitude', Float64, queue_size = 50)
AltitudeSetpoint = rospy.Publisher('/quadrotor/setpoint/altitude', Float64, queue_size = 50)

# Subscriber recieves message from topic and invokes spliter function
StateSubscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, spliter)
rospy.spin()
