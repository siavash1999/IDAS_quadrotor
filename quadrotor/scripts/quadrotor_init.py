#!/usr/bin/env python3
################################################################################
import rospy
import numpy as np
from gazebo_msgs.msg import ModelStates
from std_msgs.msg import Float64MultiArray, Float32, Float64
from geometry_msgs.msg import Pose, Twist
from tf.transformations import euler_from_quaternion
from quadrotor.qrotation import qv_mult
################################################################################
class Quadrotor():
    def __init__(self):
        self.air_frame = self.AirFrame()
        self.states = self.States()
        self.efforts = self.Efforts()

    #Class for saving physical parameters of quadrotor
    class AirFrame():
        def __init__(self):
            # Assigining Parameters
            self.m = rospy.get_param('/quadrotor/inertia/mass')
            self.Ix = rospy.get_param('/quadrotor/inertia/Ixx')
            self.Iy = rospy.get_param('/quadrotor/inertia/Iyy')
            self.Iz = rospy.get_param('/quadrotor/inertia/Izz')
            self.d = rospy.get_param('/quadrotor/geometry/d')
            self.Ct = rospy.get_param('/quadrotor/aerodynamic/ct')
            self.Cd = rospy.get_param('/quadrotor/aerodynamic/cd')
            self.Cm = rospy.get_param('/quadrotor/aerodynamic/cm')
            self.u_max = rospy.get_param('/quadrotor/u_max')

    #Class for publishing and subscribing states!
    class States():
        def __init__(self):
            #Defining Publisher and subscriber handles
            self.altitude_publisher = rospy.Publisher('/quadrotor/state/altitude', Float64, queue_size = 10)
            self.x_publisher = rospy.Publisher('/quadrotor/state/x', Float64, queue_size = 10)
            self.u_publisher = rospy.Publisher('/quadrotor/state/u', Float64, queue_size = 10)
            self.pitch_publisher = rospy.Publisher('/quadrotor/state/pitch', Float64, queue_size = 10)
            self.q_publisher = rospy.Publisher('/quadrotor/state/q', Float64, queue_size = 10)
            self.state_subscriber = rospy.Subscriber('/gazebo/model_states', ModelStates, self.spliter)

        #Defining callback function
        def spliter(self, msg):
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
            self.altitude_publisher.publish(z)
            self.x_publisher.publish(x)
            self.u_publisher.publish(u)
            self.pitch_publisher.publish(pitch)
            self.q_publisher.publish(q)
            return None

    class Efforts():
        def __init__(self):
            self.thrust = 0
            self.Tx = 0
            self.Ty = 0
            self.Tz = 0
            self.thrust_subscriber = rospy.Subscriber('/quadrotor/efforts/thrust', Float64, self.get_thrust)
            self.Tx_subscriber = rospy.Subscriber('/quadrotor/efforts/Tx', Float64, self.get_Tx)
            self.Ty_subscriber = rospy.Subscriber('/quadrotor/efforts/Ty', Float64, self.get_Ty)
            self.Tz_subscriber = rospy.Subscriber('/quadrotor/efforts/Tz', Float64, self.get_Tz)
            self.motor_command_publisher = rospy.Publisher('/quadrotor/motor_controller/command', Float64MultiArray, queue_size = 10)

        def get_thrust(self, msg):
            self.thrust = msg.data
        def get_Tx(self, msg):
            self.Tx = msg.data
        def get_Ty(self, msg):
            self.Ty = msg.data
        def get_Tz(self, msg):
            self.Tz = msg.data

        def publish_efforts(self, m, Ct, Cm, d):
            motor_vels = Float64MultiArray()
            g = 9.81
            control_allocator = 0.25*np.array([[1, -np.sqrt(2), -np.sqrt(2),  1],\
                                  [1, -np.sqrt(2),  np.sqrt(2), -1],\
                                  [1,  np.sqrt(2),  np.sqrt(2),  1],\
                                  [1,  np.sqrt(2), -np.sqrt(2), -1]])
            efforts = [(self.thrust+m*g)/Ct, self.Tx/(Ct*d), self.Ty/(Ct*d), self.Tz/Cm]
            vels_squared = np.dot(control_allocator, efforts)
            vels = np.sign(vels_squared)*np.sqrt(np.abs(vels_squared))
            vels[1] = -vels[1]
            vels[3] = -vels[3]
            motor_vels.data = vels.tolist()
            self.motor_command_publisher.publish(motor_vels)
################################################################################
rospy.init_node('quadrotor')
rate = rospy.Rate(100)
idas_quadrotor = Quadrotor()
while not rospy.is_shutdown():
    idas_quadrotor.efforts.publish_efforts(idas_quadrotor.air_frame.m, idas_quadrotor.air_frame.Ct, idas_quadrotor.air_frame.Cm, idas_quadrotor.air_frame.d)
    rate.sleep()
