#! /usr/bin/env python3
import rospy
import inverse_kinematic
from std_msgs.msg import Float32

# current_q = []

# def call_back1(msg):
#     global current_q
#     current_q.append(msg)

# def call_back2(msg):
#     global current_q
#     current_q.append(msg)

# def call_back3(msg):
#     global current_q
#     current_q.append(msg)

# def call_back4(msg):
#     global current_q
#     current_q.append(msg)

# def call_back5(msg):
#     global current_q
#     current_q.append(msg)

# def call_back6(msg):
#     global current_q
#     current_q.append(msg)

# def call_back7(msg):
#     global current_q
#     current_q.append(msg)

def main():
    rospy.init_node('franka_node')
    global current_q
    rate = rospy.Rate(10)

    desired_orientation = [[1, 0, 0],[0, 1, 0], [0, 0, 1]]
    
    #define the ros topic where to publish the joints values
    publisher1 = rospy.Publisher('Franka/joint1/cmd_vel', Float32, queue_size=10)
    publisher2 = rospy.Publisher('Franka/joint2/cmd_vel', Float32, queue_size=10)
    publisher3 = rospy.Publisher('Franka/joint3/cmd_vel', Float32, queue_size=10)
    publisher4 = rospy.Publisher('Franka/joint4/cmd_vel', Float32, queue_size=10)
    publisher5 = rospy.Publisher('Franka/joint5/cmd_vel', Float32, queue_size=10)
    publisher6 = rospy.Publisher('Franka/joint6/cmd_vel', Float32, queue_size=10)
    publisher7 = rospy.Publisher('Franka/joint7/cmd_vel', Float32, queue_size=10)

    #get the current joint config
    # sub1 = rospy.Subscriber('/Franka/joint1/state', Float32, call_back1)
    # sub2 = rospy.Subscriber('/Franka/joint2/state', Float32, call_back2)
    # sub3 = rospy.Subscriber('/Franka/joint3/state', Float32, call_back3)
    # sub4 = rospy.Subscriber('/Franka/joint4/state', Float32, call_back4)
    # sub5 = rospy.Subscriber('/Franka/joint5/state', Float32, call_back5)
    # sub6 = rospy.Subscriber('/Franka/joint6/state', Float32, call_back6)
    # sub7 = rospy.Subscriber('/Franka/joint7/state', Float32, call_back7)

    rate.sleep()

    joint1 = -3.123283e-05
    joint2 = -9.512901e-05
    joint3 = -7.152557e-07
    joint4 = -1.5708812475
    joint5 = -0.0002501010
    joint6 = 1.56948256492
    joint7 = -3.004074e-05

    current_q = [joint1, joint2, joint3, joint4, joint5, joint6, joint7]


    print(current_q)

    while not rospy.is_shutdown():
        #get the IK solution for this point
        point = [-0.07, 0, 0.413]

        q = inverse_kinematic.kuka_IK(point, desired_orientation, current_q)

        diff_q = current_q - q
        current_q = q

        #q = [0.5, 0.7, 0.4, 0.3, 0.2, 0.3, 0.5]
        print('---------------------------')
        publisher1.publish(diff_q[0])
        print(diff_q[0])
        publisher2.publish(diff_q[1])
        print(diff_q[1])
        publisher3.publish(diff_q[2])
        print(diff_q[2])
        publisher4.publish(diff_q[3])
        print(diff_q[3])
        publisher5.publish(diff_q[4])
        print(diff_q[4])
        publisher6.publish(diff_q[5])
        print(diff_q[5])
        publisher7.publish(diff_q[6])
        print(diff_q[6])
        print('---------------------------')
        rate.sleep()

if __name__ == '__main__':
    main()