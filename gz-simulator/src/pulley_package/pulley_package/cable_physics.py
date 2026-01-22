#!/usr/bin/env python3
"""
Cable Physics Controller for Cable-Driven Parallel Robot (CDPR)

Ideal cable model with no gravity - platform moves based on motor velocities.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


class CablePhysicsController(Node):
    def __init__(self):
        super().__init__('cable_physics_controller')
        
        # Movement scaling - how much platform moves per motor velocity
        self.movement_scale = 0.05
        
        # Force parameters
        self.force_gain = 50.0
        self.damping = 10.0
        
        # State variables
        self.motor_velocities = {'tl': 0.0, 'tr': 0.0, 'bl': 0.0, 'br': 0.0}
        self.platform_vx = 0.0
        self.platform_vz = 0.0
        
        # Subscribers
        self.joint_state_sub = self.create_subscription(
            JointState, '/joint_states', self.joint_state_callback, 10)
        
        # Publishers
        self.platform_effort_pub = self.create_publisher(
            Float64MultiArray, '/platform_effort_controller/commands', 10)
        
        # Control loop timer
        self.control_timer = self.create_timer(0.01, self.control_loop)  # 100 Hz
        
        self.get_logger().info('Cable Physics Controller started (ideal cables, no gravity)')
        
    def joint_state_callback(self, msg: JointState):
        """Update state from joint states."""
        for i, name in enumerate(msg.name):
            # Motor velocities
            if name == 'tl_motor_joint' and len(msg.velocity) > i:
                self.motor_velocities['tl'] = msg.velocity[i]
            elif name == 'tr_motor_joint' and len(msg.velocity) > i:
                self.motor_velocities['tr'] = msg.velocity[i]
            elif name == 'bl_motor_joint' and len(msg.velocity) > i:
                self.motor_velocities['bl'] = msg.velocity[i]
            elif name == 'br_motor_joint' and len(msg.velocity) > i:
                self.motor_velocities['br'] = msg.velocity[i]
            # Platform velocity
            elif name == 'platform_x_joint' and len(msg.velocity) > i:
                self.platform_vx = msg.velocity[i]
            elif name == 'platform_z_joint' and len(msg.velocity) > i:
                self.platform_vz = msg.velocity[i]
    
    def control_loop(self):
        """Main control loop - convert motor commands to platform forces."""
        
        tl = self.motor_velocities['tl']
        tr = self.motor_velocities['tr']
        bl = self.motor_velocities['bl']
        br = self.motor_velocities['br']
        
        # Ideal cable kinematics:
        # - Right motors (TR, BR) pull right (+X)
        # - Left motors (TL, BL) pull left (-X)  
        # - Top motors (TL, TR) pull up (+Z)
        # - Bottom motors (BL, BR) pull down (-Z)
        
        desired_vx = self.movement_scale * (tr + br - tl - bl)
        desired_vz = self.movement_scale * (tl + tr - bl - br)
        
        # PD control to achieve desired velocity
        force_x = self.force_gain * desired_vx - self.damping * self.platform_vx
        force_z = self.force_gain * desired_vz - self.damping * self.platform_vz
        
        # Clamp forces
        max_force = 100.0
        force_x = max(-max_force, min(max_force, force_x))
        force_z = max(-max_force, min(max_force, force_z))
        
        # Publish forces to platform effort controller
        effort_msg = Float64MultiArray()
        effort_msg.data = [force_x, force_z]
        self.platform_effort_pub.publish(effort_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CablePhysicsController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
