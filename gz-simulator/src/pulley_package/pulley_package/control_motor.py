#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import sys
import time

class MotorController(Node):
    def __init__(self):
        super().__init__('motor_controller')
        
        # Publisher for velocity commands
        self.velocity_pub = self.create_publisher(
            Float64MultiArray,
            '/motor_velocity_controller/commands',
            10
        )
        
        self.get_logger().info('Motor controller initialized')
    
    def set_velocity(self, velocity):
        """
        Set motor velocity in rad/s
        Positive values rotate counter-clockwise (rope winds up)
        Negative values rotate clockwise (rope unwinds)
        """
        msg = Float64MultiArray()
        msg.data = [velocity]
        self.velocity_pub.publish(msg)
        self.get_logger().info(f'Setting motor velocity to {velocity} rad/s')
    
    def stop(self):
        """Stop the motor"""
        self.set_velocity(0.0)
    
    def run_demo(self):
        """Run a demonstration sequence"""
        self.get_logger().info('Starting demo sequence...')
        time.sleep(2)
        
        # Wind up (positive velocity)
        self.get_logger().info('Winding up rope...')
        self.set_velocity(2.0)
        time.sleep(5)
        
        # Stop
        self.get_logger().info('Stopping...')
        self.stop()
        time.sleep(2)
        
        # Unwind (negative velocity)
        self.get_logger().info('Unwinding rope...')
        self.set_velocity(-2.0)
        time.sleep(5)
        
        # Stop
        self.get_logger().info('Stopping...')
        self.stop()
        
        self.get_logger().info('Demo complete!')

def main(args=None):
    rclpy.init(args=args)
    
    controller = MotorController()
    
    # Check if demo mode requested
    if len(sys.argv) > 1 and sys.argv[1] == 'demo':
        controller.run_demo()
    else:
        controller.get_logger().info('Ready to receive velocity commands')
        controller.get_logger().info('Use: ros2 topic pub /motor_velocity_controller/commands std_msgs/msg/Float64MultiArray "{data: [2.0]}"')
        rclpy.spin(controller)
    
    controller.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()