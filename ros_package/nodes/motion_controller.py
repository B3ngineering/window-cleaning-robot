#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point
from sensor_msgs.msg import JointState
from window_cleaner.msg import MotorCommand, RobotState, Stop
import math
import numpy as np

# Motor indices matching MotorCommand.msg and anchor layout
MOTOR_TL = 0  # Top-Left
MOTOR_TR = 1  # Top-Right
MOTOR_BL = 2  # Bottom-Left
MOTOR_BR = 3  # Bottom-Right


class MotionController:
    def __init__(self):
        rospy.init_node('motion_controller')

        # Load parameters from robot_params.yaml (loaded to root namespace)
        frame_width = rospy.get_param('/pulley_separation_x', 1.56)
        frame_height = rospy.get_param('/pulley_separation_y', 1.32)
        robot_width = rospy.get_param('/robot_width', 0.2)
        robot_height = rospy.get_param('/robot_height', 0.3)
        spool_radius = rospy.get_param('/spool_radius', 0.05)
        steps_per_rev = rospy.get_param('/steps_per_rev', 3200)
        
        self.position_tolerance = rospy.get_param('/position_tolerance', 0.01)
        self.max_speed_rpm = rospy.get_param('/max_speed_rpm', 3000)

        # Initialize cable geometry calculator
        self.cable_geom = CableGeometry(
            frame_width=frame_width,
            frame_height=frame_height,
            robot_width=robot_width,
            robot_height=robot_height,
            spool_radius=spool_radius,
            steps_per_rev=int(steps_per_rev)
        )

        # State tracking
        self.target = None
        self.stop_active = False
        self.last_stop_time = rospy.Time.now()
        self.status = RobotState.STATUS_IDLE
        
        # Current position and cable state
        # Initialize to workspace center (assumed starting position)
        self.current_position = np.array([frame_width / 2, frame_height / 2])
        self.current_cable_lengths = self.cable_geom.position_to_cable_lengths(self.current_position)
        
        # Encoder tracking: we track deltas from initial position
        # At startup, encoders read some value which corresponds to initial_cable_lengths
        self.initial_encoder_steps = None  # Will be set on first joint_state message
        self.current_steps = np.zeros(4, dtype=int)

        # Publishers
        self.motor_pub = rospy.Publisher('/motor_commands', MotorCommand, queue_size=10)
        self.state_pub = rospy.Publisher('/robot_state', RobotState, queue_size=10)

        # Subscribers
        rospy.Subscriber('/target_position', Point, self.target_cb)
        rospy.Subscriber('/joint_states', JointState, self.joint_state_cb)
        rospy.Subscriber('/emergency_stop', Stop, self.stop_cb)

        # State publish timer
        publish_rate = rospy.get_param('~state_publish_rate', 10.0)
        rospy.Timer(rospy.Duration(1.0 / publish_rate), self.publish_state)
        
        rospy.loginfo(f"Motion Controller initialized")
        rospy.loginfo(f"  Frame: {frame_width}m x {frame_height}m")
        rospy.loginfo(f"  Robot: {robot_width}m x {robot_height}m")
        rospy.loginfo(f"  Workspace: {self.cable_geom.workspace_min} to {self.cable_geom.workspace_max}")

    def target_cb(self, msg):
        """Handle new target position from navigation node."""
        if self.stop_active:
            rospy.logwarn("Cannot move: emergency stop is active")
            return
            
        target_pos = np.array([msg.x, msg.y])
        
        # Validate target is within workspace
        if not self.cable_geom.is_in_workspace(target_pos):
            rospy.logerr(f"Target {target_pos} is outside workspace bounds "
                        f"[{self.cable_geom.workspace_min}, {self.cable_geom.workspace_max}]")
            return
        
        self.target = target_pos
        rospy.loginfo(f"Moving to target: ({msg.x:.3f}, {msg.y:.3f})")
        
        # Compute required cable lengths for target position
        target_lengths = self.cable_geom.position_to_cable_lengths(target_pos)
        
        # Compute delta steps from current to target
        delta_steps = self.cable_geom.compute_delta_steps(self.current_cable_lengths, target_lengths)
        
        rospy.loginfo(f"  Current lengths: {self.current_cable_lengths}")
        rospy.loginfo(f"  Target lengths:  {target_lengths}")
        rospy.loginfo(f"  Delta steps:     {delta_steps}")
        
        # Send motor commands
        self.status = RobotState.STATUS_MOVING
        self.send_motor_commands(delta_steps)

    def send_motor_commands(self, delta_steps: np.ndarray):
        """
        Send coordinated motor commands so all motors finish at the same time.
        
        The motor with the largest delta runs at max_speed_rpm.
        Other motors run proportionally slower so all finish together.
        """
        abs_steps = np.abs(delta_steps)
        max_steps = np.max(abs_steps)
        
        if max_steps == 0:
            rospy.loginfo("No movement required")
            return
        
        # Calculate proportional speeds for each motor
        # Motor with max steps runs at max_speed, others scaled proportionally
        speeds = np.zeros(4)
        for i in range(4):
            if abs_steps[i] > 0:
                # Speed proportional to steps needed (so all finish at same time)
                speeds[i] = (abs_steps[i] / max_steps) * self.max_speed_rpm
            else:
                speeds[i] = 0
        
        rospy.loginfo(f"  Coordinated speeds: {speeds} RPM")
        
        # Send commands to all motors
        for motor_id in range(4):
            cmd = MotorCommand()
            cmd.Header.stamp = rospy.Time.now()
            cmd.motor_id = motor_id
            cmd.steps = int(abs_steps[motor_id])
            cmd.speed_rpm = float(speeds[motor_id])
            # Positive delta = cable needs to get longer = let out = -1
            # Negative delta = cable needs to get shorter = reel in = +1
            cmd.direction = -1 if delta_steps[motor_id] > 0 else 1
            cmd.stop = False
            
            self.motor_pub.publish(cmd)
            rospy.logdebug(f"Motor {motor_id}: {cmd.steps} steps @ {cmd.speed_rpm:.1f} RPM, dir={cmd.direction}")

    def halt_all_motors(self):
        """Immediately stop all motors (emergency halt)."""
        rospy.logwarn("HALT: Stopping all motors")
        for motor_id in range(4):
            cmd = MotorCommand()
            cmd.Header.stamp = rospy.Time.now()
            cmd.motor_id = motor_id
            cmd.steps = 0
            cmd.speed_rpm = 0
            cmd.direction = 0
            cmd.stop = True
            self.motor_pub.publish(cmd)
        self.target = None
        self.status = RobotState.STATUS_STOP

    def joint_state_cb(self, msg):
        """Process encoder feedback from motor_stm32_bridge to update position."""
        # JointState.position contains encoder step counts for each motor
        # Expected order: [TL, TR, BL, BR]
        if len(msg.position) < 4:
            rospy.logwarn(f"Incomplete joint states: expected 4, got {len(msg.position)}")
            return
        
        encoder_steps = np.array(msg.position[:4], dtype=int)
        
        # On first message, capture initial encoder values as reference
        if self.initial_encoder_steps is None:
            self.initial_encoder_steps = encoder_steps.copy()
            rospy.loginfo(f"Encoder reference set: {self.initial_encoder_steps}")
            return
        
        # Compute step deltas from initial position
        delta_steps = encoder_steps - self.initial_encoder_steps
        self.current_steps = encoder_steps
        
        # Convert step deltas to cable length deltas
        # initial_cable_lengths + delta_lengths = current_cable_lengths
        initial_lengths = self.cable_geom.position_to_cable_lengths(
            np.array([self.cable_geom.frame_width / 2, self.cable_geom.frame_height / 2])
        )
        
        for i in range(4):
            delta_length = self.cable_geom.steps_to_cable_length(delta_steps[i])
            self.current_cable_lengths[i] = initial_lengths[i] + delta_length
        
        # Compute current position from cable lengths (forward kinematics)
        position = self.cable_geom.cable_lengths_to_position(self.current_cable_lengths)
        if position is not None:
            self.current_position = position
            
            # Check if we've reached target
            if self.target is not None:
                distance = np.linalg.norm(self.current_position - self.target)
                if distance < self.position_tolerance:
                    rospy.loginfo(f"Reached target! Distance: {distance:.4f}m")
                    self.status = RobotState.STATUS_AT_TARGET
                    self.target = None
        else:
            rospy.logwarn("Forward kinematics failed - singular configuration")

    def stop_cb(self, msg):
        """Handle emergency stop signals."""
        self.stop_active = msg.active
        if self.stop_active:
            self.last_stop_time = rospy.Time.now()
            rospy.logwarn(f"Emergency stop activated! - {msg.reason}")
            self.halt_all_motors()  # Immediately stop all motors
        else:
            self.status = RobotState.STATUS_IDLE
            rospy.loginfo("Emergency stop deactivated")

    def publish_state(self, event):
        """Publish current robot state at regular intervals."""
        state = RobotState()
        state.header.stamp = rospy.Time.now()
        
        # Position
        state.x = float(self.current_position[0])
        state.y = float(self.current_position[1])
        
        # Cable lengths
        state.cable_length_tl = float(self.current_cable_lengths[MOTOR_TL])
        state.cable_length_tr = float(self.current_cable_lengths[MOTOR_TR])
        state.cable_length_bl = float(self.current_cable_lengths[MOTOR_BL])
        state.cable_length_br = float(self.current_cable_lengths[MOTOR_BR])
        
        # Motor steps
        state.steps_motor_tl = int(self.current_steps[MOTOR_TL])
        state.steps_motor_tr = int(self.current_steps[MOTOR_TR])
        state.steps_motor_bl = int(self.current_steps[MOTOR_BL])
        state.steps_motor_br = int(self.current_steps[MOTOR_BR])
        
        # Status
        state.status = self.status

        self.state_pub.publish(state)


class CableGeometry:
    """
    Cable kinematics for our specific robot configuration.
    The platform is a rectangle with four cables attached to its corners.
    These attachments are on a sliding mechanism that prevents rotation.

    Coordinate system:
    - Origin at bottom-left anchor
    - X-axis to the right, Y-axis upwards

    Anchor and attachment pairings:
    - Anchor 0 (TL) <- Cable 0 -> Attachment 0 (TL)
    - Anchor 1 (TR) <- Cable 1 -> Attachment 1 (TR)
    - Anchor 2 (BL) <- Cable 2 -> Attachment 2 (BL)
    - Anchor 3 (BR) <- Cable 3 -> Attachment 3 (BR)
    """

    def __init__(self, frame_width: float, frame_height: float, robot_width: float, robot_height: float,
                 spool_radius: float, steps_per_rev: int):
        """
        Args:
            frame_width: Distance between left and right anchors
            frame_height: Distance between top and bottom anchors
            robot_width: Distance between left and right attachments on the platform
            robot_height: Distance between top and bottom attachments on the platform
            spool_radius: Radius of the motor spool (for converting steps to cable length)
            steps_per_rev: Number of motor steps per full revolution (for converting steps to cable length)
        """
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.robot_width = robot_width
        self.robot_height = robot_height
        self.spool_radius = spool_radius
        self.steps_per_rev = steps_per_rev

        # Define anchor positions based on frame dimensions
        self.anchors = np.array([
            [0.0, frame_height],  # TL
            [frame_width, frame_height],  # TR
            [0.0, 0.0],  # BL
            [frame_width, 0.0]  # BR
        ])

        # Now define attachment positions relative to the robot's center (x, y)
        self.attachments = np.array([
            [-robot_width / 2, robot_height / 2],  # TL
            [robot_width / 2, robot_height / 2],  # TR
            [-robot_width / 2, -robot_height / 2],  # BL
            [robot_width / 2, -robot_height / 2]  # BR
        ])

        # Compute workspace boundaries based on anchor and attachment geometry
        self.workspace_min = np.array([robot_width / 2, robot_height / 2])  # Minimum (x, y) at center of robot
        self.workspace_max = np.array([frame_width - robot_width / 2, frame_height - robot_height / 2])  # Maximum (x, y) at center of robot

    def get_attachment_point(self, robot_center: np.ndarray, attachment_index: int) -> np.ndarray:
        """
        Get the world coordinates of a specific attachment point based on the robot's center position.
        We need this to calculate cable lengths and for inverse kinematics calculations.
        """
        return robot_center + self.attachments[attachment_index]
    
    def position_to_cable_lengths(self, robot_center: np.ndarray) -> np.ndarray:
        """
        Inverse kinematics function
        Given the robot's center position, calculate the required length of all four cables.
        """

        lengths = np.zeros(4)
        for i in range(4):
            attachment_point = self.get_attachment_point(robot_center, i)
            lengths[i] = np.linalg.norm(self.anchors[i] - attachment_point)

        return lengths
    
    def cable_lengths_to_position(self, lengths: np.ndarray) -> np.ndarray:
        """
        Forward kinematics function
        Given the lengths of all four cables, calculate the position of the robot's center.
        This is a simplified version and may not be numerically stable for all configurations.
        """

        # Convert the problem back to point mass
        anchor_points = self.anchors - self.attachments  # Shift anchors to attachment frame
        
        A = np.array([
            [2 * (anchor_points[2][0] - anchor_points[0][0]), 2 * (anchor_points[2][1] - anchor_points[0][1])],
            [2 * (anchor_points[2][0] - anchor_points[1][0]), 2 * (anchor_points[2][1] - anchor_points[1][1])],
        ])

        b = np.array([
            lengths[0]**2 - lengths[2]**2 - np.dot(anchor_points[0], anchor_points[0]) + np.dot(anchor_points[2], anchor_points[2]),
            lengths[1]**2 - lengths[2]**2 - np.dot(anchor_points[1], anchor_points[1]) + np.dot(anchor_points[2], anchor_points[2])
        ])

        try:
            position = np.linalg.solve(A, b)
            return position
        except np.linalg.LinAlgError:
                    rospy.logerr("Singular configuration detected in cable_lengths_to_position")
                    return None
        
    def is_in_workspace(self, position: np.ndarray) -> bool:
        """
        Check if a given position is within the robot's workspace.
        This is important for validating target positions before attempting to move there.
        """
        return (self.workspace_min[0] <= position[0] <= self.workspace_max[0] and
                self.workspace_min[1] <= position[1] <= self.workspace_max[1])

    def cable_length_to_steps(self, length: float) -> int:
        """
        Convert a cable length to motor steps based on the spool radius and steps per revolution.
        """
        circumference = 2 * math.pi * self.spool_radius
        revolutions = length / circumference
        return int(revolutions * self.steps_per_rev)
    
    def steps_to_cable_length(self, steps: int) -> float:
        """
        Convert motor steps back to cable length.
        """
        circumference = 2 * math.pi * self.spool_radius
        revolutions = steps / self.steps_per_rev
        return revolutions * circumference
    
    def compute_delta_steps(self, current_lengths: np.ndarray, target_lengths: np.ndarray) -> np.ndarray:
        """
        Given current and target cable lengths, calculate the delta in steps for each motor.
        Positive steps indicate the motor should reel in (shorten the cable), negative steps indicate it should let out (lengthen the cable).
        """
        delta_lengths = target_lengths - current_lengths
        delta_steps = np.zeros(4, dtype=int)
        for i in range(4):
            delta_steps[i] = self.cable_length_to_steps(delta_lengths[i])

        return delta_steps
    
    def jacobian(self, position: np.ndarray) -> np.ndarray:
        """
        Compute the Jacobian matrix for the current position.
        """
        J = np.zeros((4, 2))
        for i in range(4):
            attachment_point = self.get_attachment_point(position, i)
            diff = attachment_point - self.anchors[i]
            length = np.linalg.norm(diff)
            if length > 0:
                J[i, :] = diff / length
            else:
                rospy.logerr("Zero-length cable detected in Jacobian calculation")
                J[i, :] = 0.0

        return J
    

if __name__ == '__main__':
    try:
        MotionController()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass