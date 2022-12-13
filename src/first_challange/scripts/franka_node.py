#! /usr/bin/env python3
import time
import math
import rospy
import ik
from controller import Controller
from std_msgs.msg import Float32

def main():
    '''
    This function is the main function that is called when the node is started.
    '''

    #init node
    rospy.init_node('franka_node')

    ros_controller = Controller()
    global current_q
    rate = rospy.Rate(10)
    
    #define the ros topic where to publish the joints value

    # Start joint state
    joint1 = (0)
    joint2 = (0)
    joint3 = (0)
    joint4 = (-math.pi/2)
    joint5 = (0)
    joint6 = (math.pi/2)
    joint7 = (0)

    #set the current q
    current_q = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]

    while not rospy.is_shutdown():

        time.sleep(3)


        all_zeros = [0, 0, 0, 0, 0, 0, 0]
        start = [0, 0, 0, -math.pi/2, 0, -math.pi/2, 0]

        # Sets the robot to all zeros
        ros_controller.moveToPosition(all_zeros)

        # Sets the robot to starting position
        ros_controller.moveToPosition(start)

        time.sleep(3)

        # get solution to this point
        point = [0.413, 0, -0.07, 1, 1, 1]

        #q = inverse_kinematic.kuka_IK(point, current_q)
        q = ik.calculate_inverse_kinematics(0.01)

        # set the current q to the old q
        current_q = q

        #Publish the results
        ros_controller.moveToPosition(q)

        break


if __name__ == '__main__':
    main()