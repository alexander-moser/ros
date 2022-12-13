import rospy
from std_msgs.msg import Float32

class Controller:
    '''
    This class is resposible for handling all communication
    between ros and coppeliaSim
    '''

    def __init__(self):
        self.publisher1 = rospy.Publisher('Franka/joint1/cmd_vel', Float32, queue_size=10)
        self.publisher2 = rospy.Publisher('Franka/joint2/cmd_vel', Float32, queue_size=10)
        self.publisher3 = rospy.Publisher('Franka/joint3/cmd_vel', Float32, queue_size=10)
        self.publisher4 = rospy.Publisher('Franka/joint4/cmd_vel', Float32, queue_size=10)
        self.publisher5 = rospy.Publisher('Franka/joint5/cmd_vel', Float32, queue_size=10)
        self.publisher6 = rospy.Publisher('Franka/joint6/cmd_vel', Float32, queue_size=10)
        self.publisher7 = rospy.Publisher('Franka/joint7/cmd_vel', Float32, queue_size=10)

    def moveToPosition(self, position):
        '''
        Function that takes position and publishes them
        to all the links
        '''

        self.publisher1.publish(position[0])
        self.publisher2.publish(position[1])
        self.publisher3.publish(position[2])
        self.publisher4.publish(position[3])
        self.publisher5.publish(position[4])
        self.publisher6.publish(position[5])
        self.publisher7.publish(position[6])
