#! /usr/bin/env python3

import rospy
import inverse_kinematic
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
from std_srvs.srv import EmptyResponse, EmptyRequest, Empty

def main():
    rospy.init_node('franka_node')
    print(rospy.is_shutdown())
    rate = rospy.Rate(500)

    desired_orientation = [[0, 0, -1], [0, 1, 0], [1, 0, 0]]

    current_q = [0, 1.12, 0, 1.71, 0, 1.84, 0]

    #the joint names of kuka:
    joint_names = ['joint1', 'joint2', 'joint3', 'joint4', 'joint5', 'joint6', 'joint7']
    #define the ros message for publishing the joint positions
    joint_msg = Float32()
    
    #define the ros topic where to publish the joints values
    publisher1 = rospy.Publisher('Franka/joint1/cmd_vel', Float32, queue_size=10)
    publisher2 = rospy.Publisher('Franka/joint2/cmd_vel', Float32, queue_size=10)
    publisher3 = rospy.Publisher('Franka/joint3/cmd_vel', Float32, queue_size=10)
    publisher4 = rospy.Publisher('Franka/joint4/cmd_vel', Float32, queue_size=10)
    publisher5 = rospy.Publisher('Franka/joint5/cmd_vel', Float32, queue_size=10)
    publisher6 = rospy.Publisher('Franka/joint6/cmd_vel', Float32, queue_size=10)
    publisher7 = rospy.Publisher('Franka/joint7/cmd_vel', Float32, queue_size=10)

    print(rospy.is_shutdown())

    while not rospy.is_shutdown():
        #get the IK solution for this point
        point = [5, 0, 0]

        q = inverse_kinematic.kuka_IK(point, desired_orientation, current_q)

        current_q = q
        q_msg = [q[0], 0, q[1], 0, q[2], 0, q[3], 0, q[4], 0, q[5], 0, q[6], 0]
        #print(q_msg)
        #publish this solution
        #joint_msg = q_msg

        publisher1.publish(q[0])
        publisher2.publish(q[1])
        publisher3.publish(q[2])
        publisher4.publish(q[3])
        publisher5.publish(q[4])
        publisher6.publish(q[5])
        publisher7.publish(q[6])

        #publish the path to be visualized in r
        # viz
        rate.sleep()



if __name__ == '__main__':
    main()