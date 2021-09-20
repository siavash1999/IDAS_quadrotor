#!/usr/bin/env python3
################################################################################
import rospy
from std_msgs.msg import Float64MultiArray, Float32, Float64
from gazebo_msgs.msg import LinkStates
from geometry_msgs.msg import Pose, Twist
################################################################################
def spliter_2(msg):
    index = msg.name.index('quadrotor::propeller_1')
    prop_speed = msg.twist[index].angular.z
    prop_pub.publish(prop_speed)
################################################################################
rospy.init_node('propeller_publisher')
rate = rospy.Rate(100)

while not rospy.is_shutdown():
    prop_pub = rospy.Publisher('/quadrotor/propeller_speed', Float64, queue_size = 10)
    prop_sub = rospy.Subscriber('/gazebo/link_states', LinkStates, spliter_2)
    rate.sleep()
