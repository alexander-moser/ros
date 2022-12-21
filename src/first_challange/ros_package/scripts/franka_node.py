#! /usr/bin/env python3
import numpy as np
import sympy as sp
import rospy
import threading
from controller import Controller
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
import forward_kinematics
import inverse_kinematics
import math

current_q = []

#callbackfunctions for retrieving the current joint state
def call_back1(msg):
    global current_q
    current_q[0] = msg.data

def call_back2(msg):
    global current_q
    current_q[1] = msg.data

def call_back3(msg):
    global current_q
    current_q[2] = msg.data

def call_back4(msg):
    global current_q
    current_q[3] = msg.data

def call_back5(msg):
    global current_q
    current_q[4] = msg.data

def call_back6(msg):
    global current_q
    current_q[5] = msg.data

def call_back7(msg):
    global current_q
    current_q[6] = msg.data

def goal_callback(msg):
    global Goal_Position
    Goal_Position = [msg.linear.x, msg.linear.y, msg.linear.z, msg.angular.x, msg.angular.y, msg.angular.z]


def state_listener():
    #get the current joint config
    sub1 = rospy.Subscriber('/Franka/joint1/state', Float32, call_back1)
    sub2 = rospy.Subscriber('/Franka/joint2/state', Float32, call_back2)
    sub3 = rospy.Subscriber('/Franka/joint3/state', Float32, call_back3)
    sub4 = rospy.Subscriber('/Franka/joint4/state', Float32, call_back4)
    sub5 = rospy.Subscriber('/Franka/joint5/state', Float32, call_back5)
    sub6 = rospy.Subscriber('/Franka/joint6/state', Float32, call_back6)
    sub7 = rospy.Subscriber('/Franka/joint7/state', Float32, call_back7)
    sub_goal = rospy.Subscriber('/goal', Twist, goal_callback)
    rospy.spin()

if __name__ == '__main__':

    #init node
    rospy.init_node('franka_node')
    rate = rospy.Rate(10)       

    #setup a watchdog to retriev the current joint configuration
    state_listener = threading.Thread(target=state_listener, daemon = True)
    state_listener.start()

    #using sympy instead of numpy, so sin(pi) will actually be zero, and not just a very small value
    pi = sp.pi

    #some parameters
    ros_controller = Controller()
    current_q = [0,0,0,(-pi/2).evalf(),0,(pi/2).evalf(),0]
    stop = [0.0] * 7
    rate = rospy.Rate(10)
    tolerance = 0.01
    learning_rate = 0.2

    # dh parameters
    d = [0.3330, 0.0000, 0.3160, 0.0000, 0.3840, 0.0000, 0.1070]
    a = [0.0000, 0.0000, 0.0825, 0.0825, 0.0000, 0.0880, 0.0000]
    alpha = [pi/2, pi/2, pi/2, pi/2, pi/2, pi/2, 0]

    #calulate the first endeffector position
    theta = [current_q[0]+pi, current_q[1]-pi, current_q[2], current_q[3]+pi, current_q[4]-pi, current_q[5], current_q[6]]
    total_t_matrix = forward_kinematics.calculate_total_t_matrix(theta, d, a, alpha);
    Current_Endeffector_Position = forward_kinematics.calculate_end_effector_position(total_t_matrix);

    #just move a little downwards
    Goal_Position = np.array(np.float64(Current_Endeffector_Position)) - [0,0,0.25,0,0,0]

    while not rospy.is_shutdown():

        #actualize current theta/q
        theta = [current_q[0]+pi, current_q[1]-pi, current_q[2], current_q[3]+pi, current_q[4]-pi, current_q[5], current_q[6]]

        #forward kinematics getting endeffector_position
        total_t_matrix = forward_kinematics.calculate_total_t_matrix(theta, d, a, alpha);
        Current_Endeffector_Position = forward_kinematics.calculate_end_effector_position(total_t_matrix);

        #calculating the error
        error = np.array(Current_Endeffector_Position) - np.array(Goal_Position)
        error = np.float64(error)

        #if below tolerance, sending stop signal to joints
        if (np.linalg.norm(error) < tolerance): 
            ros_controller.moveToPosition(stop)
            continue

        #calculating the necassary velocity
        vel = - learning_rate * (inverse_kinematics.calculate_analytic_jacobian_pseudo_invers(theta, d, a, alpha) @ error)

        #oredering to move
        ros_controller.moveToPosition(vel)
        rate.sleep()