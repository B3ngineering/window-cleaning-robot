#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msg.msg import JointState
from window_cleaner.msg import MotorCommand, RobotState, Stop

class MotionController:
    def __init__(self):
        rospy.init_node('motion_controller')

        # State
        self.target = None
        self.stop_active = False
        self.last_stop_time = rospy.Time.now()

        # Publishers
        self.motor_pub = rospy.Publisher('/motor_commands', MotorCommand, queue_size=10)