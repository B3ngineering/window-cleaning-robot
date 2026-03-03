#!/usr/bin/env python3
import rospy
from sensor_msgs.msg import JointState
from window_cleaner.msg import MotorCommand

class MockMotorSTM32Bridge:
    def __init__(self):
        rospy.init_node('motor_stm32_bridge')  # Same name as real node

        self.step_counts = [0, 0, 0, 0]

        self.joint_state_pub = rospy.Publisher('/joint_states', JointState, queue_size=10)
        rospy.Subscriber('/motor_commands', MotorCommand, self.motor_command_cb)

        rospy.loginfo("MOCK motor_stm32_bridge ready")

    def motor_command_cb(self, msg):
        if 0 <= msg.motor_id <= 3:
            self.step_counts[msg.motor_id] += msg.steps * msg.direction
            rospy.loginfo(f"MOCK bridge: motor {msg.motor_id} -> "
                          f"cumulative steps {self.step_counts[msg.motor_id]}")

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = ['motor_tl', 'motor_tr', 'motor_bl', 'motor_br']
        js.position = [float(s) for s in self.step_counts]
        self.joint_state_pub.publish(js)

if __name__ == '__main__':
    MockMotorSTM32Bridge()
    rospy.spin()