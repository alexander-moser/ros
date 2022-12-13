#! /usr/bin/env python3
import time
import math
import rospy
import inverse_kinematic
import ik
from std_msgs.msg import Float32

def main():
    '''
    This function is the main function that is called when the node is started.
    '''

    #init node
    rospy.init_node('franka_node')
    global current_q
    rate = rospy.Rate(10)
    
    #define the ros topic where to publish the joints values
    publisher1 = rospy.Publisher('Franka/joint1/cmd_vel', Float32, queue_size=10)
    publisher2 = rospy.Publisher('Franka/joint2/cmd_vel', Float32, queue_size=10)
    publisher3 = rospy.Publisher('Franka/joint3/cmd_vel', Float32, queue_size=10)
    publisher4 = rospy.Publisher('Franka/joint4/cmd_vel', Float32, queue_size=10)
    publisher5 = rospy.Publisher('Franka/joint5/cmd_vel', Float32, queue_size=10)
    publisher6 = rospy.Publisher('Franka/joint6/cmd_vel', Float32, queue_size=10)
    publisher7 = rospy.Publisher('Franka/joint7/cmd_vel', Float32, queue_size=10)


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

        # Sets the robot to all zeros
        publisher1.publish(0)
        publisher2.publish(0)
        publisher3.publish(0)
        publisher4.publish(0)
        publisher5.publish(0)
        publisher6.publish(0)
        publisher7.publish(0)

        time.sleep(3)

        # Sets the robot to starting position
        publisher1.publish(0)
        publisher2.publish(0)
        publisher3.publish(0)
        publisher4.publish(-math.pi/2)
        publisher5.publish(0)
        publisher6.publish(math.pi/2)
        publisher7.publish(0)

        time.sleep(3)

        # get solution to this point
        point = [0.413, 0, -0.07, 1, 1, 1]

        #q = inverse_kinematic.kuka_IK(point, current_q)
        q = ik.ik(0.1)

        # set the current q to the old q
        current_q = q

        #Publish the results
        publisher1.publish(q[0])
        publisher2.publish(q[1])
        publisher3.publish(q[2])
        publisher4.publish(q[3])
        publisher5.publish(q[4])
        publisher6.publish(q[5])
        publisher7.publish(q[6])

        break


if __name__ == '__main__':
    main()