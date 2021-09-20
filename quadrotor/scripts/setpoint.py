#!/usr/bin/env python3
################################################################################
import rospy
from std_msgs.msg import Float64MultiArray, Float32, Float64
################################################################################
rospy.init_node('setpoint')
setpoint_altitude = rospy.Publisher('/quadrotor/setpoint/altitude', Float64, queue_size = 10)
setpoint_x = rospy.Publisher('/quadrotor/setpoint/x', Float64, queue_size = 10)
setpoint_pitch = rospy.Publisher('/quadrotor/setpoint/pitch', Float64, queue_size = 10)
rate = rospy.Rate(100)
while not rospy.is_shutdown():
    setpoint_altitude.publish(rospy.get_param('/quadrotor/setpoint/altitude'))
    setpoint_x.publish(rospy.get_param('/quadrotor/setpoint/x'))
    setpoint_pitch.publish(rospy.get_param('/quadrotor/setpoint/pitch'))
    rate.sleep()
