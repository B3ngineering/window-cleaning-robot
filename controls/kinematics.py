"""
Cable robot kinematics for window cleaning robot.

Provides forward and inverse kinematics for a 4-cable parallel robot
with rectangular platform and corner attachments.

Coordinate system:
- Origin at bottom-left anchor
- X-axis to the right, Y-axis upwards

Anchor layout:
    TL (0) -------- TR (1)
       |            |
       |   [robot]  |
       |            |
    BL (2) -------- BR (3)
"""

import math
import numpy as np
from dataclasses import dataclass
from typing import Optional, Tuple

# Motor indices matching anchor layout
MOTOR_TL = 0  # Top-Left
MOTOR_TR = 1  # Top-Right
MOTOR_BL = 2  # Bottom-Left
MOTOR_BR = 3  # Bottom-Right


@dataclass
class RobotConfig:
    """Robot configuration parameters."""
    frame_width: float
    frame_height: float
    robot_width: float
    robot_height: float
    spool_radius: float
    steps_per_rev: int
    safety_margin: float = 0.1

    @classmethod
    def from_dict(cls, config: dict) -> 'RobotConfig':
        """Create RobotConfig from config dictionary."""
        return cls(
            frame_width=config['frame']['width'],
            frame_height=config['frame']['height'],
            robot_width=config['robot']['width'],
            robot_height=config['robot']['height'],
            spool_radius=config['motor']['spool_radius'],
            steps_per_rev=config['motor']['steps_per_rev'],
            safety_margin=config.get('motion', {}).get('safety_margin', 0.1)
        )


class CableGeometry:
    """
    Cable kinematics for 4-cable parallel robot.
    
    The platform is a rectangle with four cables attached to its corners.
    Each cable connects a frame anchor to the corresponding corner of the robot.
    """

    def __init__(self, config: RobotConfig):
        """
        Initialize cable geometry calculator.
        
        Args:
            config: RobotConfig with frame and robot dimensions
        """
        self.config = config
        
        # Define anchor positions (frame corners)
        self.anchors = np.array([
            [0.0, config.frame_height],                    # TL
            [config.frame_width, config.frame_height],    # TR
            [0.0, 0.0],                                    # BL
            [config.frame_width, 0.0]                      # BR
        ])

        # Attachment offsets relative to robot center
        self.attachments = np.array([
            [-config.robot_width / 2,  config.robot_height / 2],   # TL
            [ config.robot_width / 2,  config.robot_height / 2],   # TR
            [-config.robot_width / 2, -config.robot_height / 2],   # BL
            [ config.robot_width / 2, -config.robot_height / 2]    # BR
        ])

        # Workspace boundaries (robot center must stay within these)
        margin = config.safety_margin
        self.workspace_min = np.array([
            config.robot_width / 2 + margin,
            config.robot_height / 2 + margin
        ])
        self.workspace_max = np.array([
            config.frame_width - config.robot_width / 2 - margin,
            config.frame_height - config.robot_height / 2 - margin
        ])

    def get_attachment_point(self, robot_center: np.ndarray, index: int) -> np.ndarray:
        """Get world coordinates of attachment point given robot center position."""
        return robot_center + self.attachments[index]

    def position_to_cable_lengths(self, robot_center: np.ndarray) -> np.ndarray:
        """
        Inverse kinematics: position -> cable lengths.
        
        Args:
            robot_center: (x, y) position of robot center
            
        Returns:
            Array of 4 cable lengths [TL, TR, BL, BR]
        """
        lengths = np.zeros(4)
        for i in range(4):
            attachment = self.get_attachment_point(robot_center, i)
            lengths[i] = np.linalg.norm(self.anchors[i] - attachment)
        return lengths

    def cable_lengths_to_position(self, lengths: np.ndarray) -> Optional[np.ndarray]:
        """
        Forward kinematics: cable lengths -> position.
        
        Uses trilateration with first 3 cables (over-constrained system).
        
        Args:
            lengths: Array of 4 cable lengths
            
        Returns:
            (x, y) position of robot center, or None if singular
        """
        # Shift anchors by attachment offsets to solve for robot center
        anchor_points = self.anchors - self.attachments
        
        # Set up linear system from 2 equations (using cables 0, 1, 2)
        A = np.array([
            [2 * (anchor_points[2][0] - anchor_points[0][0]),
             2 * (anchor_points[2][1] - anchor_points[0][1])],
            [2 * (anchor_points[2][0] - anchor_points[1][0]),
             2 * (anchor_points[2][1] - anchor_points[1][1])],
        ])

        b = np.array([
            lengths[0]**2 - lengths[2]**2 
            - np.dot(anchor_points[0], anchor_points[0]) 
            + np.dot(anchor_points[2], anchor_points[2]),
            lengths[1]**2 - lengths[2]**2 
            - np.dot(anchor_points[1], anchor_points[1]) 
            + np.dot(anchor_points[2], anchor_points[2])
        ])

        try:
            position = np.linalg.solve(A, b)
            return position
        except np.linalg.LinAlgError:
            return None

    def is_in_workspace(self, position: np.ndarray) -> bool:
        """Check if position is within valid workspace bounds."""
        return (self.workspace_min[0] <= position[0] <= self.workspace_max[0] and
                self.workspace_min[1] <= position[1] <= self.workspace_max[1])

    def validate_position(self, x: float, y: float) -> Tuple[bool, str]:
        """
        Validate target position.
        
        Returns:
            (is_valid, message)
        """
        pos = np.array([x, y])
        
        if not self.is_in_workspace(pos):
            return False, (f"Position ({x:.3f}, {y:.3f}) outside workspace "
                          f"[{self.workspace_min} to {self.workspace_max}]")
        
        # Check all cable lengths are positive (sanity check)
        lengths = self.position_to_cable_lengths(pos)
        if np.any(lengths <= 0):
            return False, f"Invalid cable lengths: {lengths}"
        
        return True, "OK"

    def cable_length_to_steps(self, length: float) -> int:
        """Convert cable length (meters) to motor steps."""
        circumference = 2 * math.pi * self.config.spool_radius
        revolutions = length / circumference
        return int(revolutions * self.config.steps_per_rev)

    def steps_to_cable_length(self, steps: int) -> float:
        """Convert motor steps to cable length (meters)."""
        circumference = 2 * math.pi * self.config.spool_radius
        revolutions = steps / self.config.steps_per_rev
        return revolutions * circumference

    def cable_lengths_to_steps(self, lengths: np.ndarray) -> np.ndarray:
        """Convert array of cable lengths to motor steps."""
        return np.array([self.cable_length_to_steps(l) for l in lengths], dtype=int)

    def steps_to_cable_lengths(self, steps: np.ndarray) -> np.ndarray:
        """Convert array of motor steps to cable lengths."""
        return np.array([self.steps_to_cable_length(s) for s in steps])

    def compute_delta_steps(self, current_lengths: np.ndarray, 
                           target_lengths: np.ndarray) -> np.ndarray:
        """
        Compute step deltas needed to move from current to target position.
        
        Returns:
            Array of step deltas for each motor (positive = lengthen cable)
        """
        delta_lengths = target_lengths - current_lengths
        return np.array([self.cable_length_to_steps(dl) for dl in delta_lengths], dtype=int)

    def compute_motor_speeds(self, delta_steps: np.ndarray, 
                            max_speed_rpm: float) -> np.ndarray:
        """
        Compute coordinated motor speeds so all motors finish simultaneously.
        
        The motor with the largest delta runs at max_speed_rpm.
        Other motors run proportionally slower.
        
        Returns:
            Array of speeds in RPM for each motor
        """
        abs_steps = np.abs(delta_steps)
        max_steps = np.max(abs_steps)
        
        if max_steps == 0:
            return np.zeros(4)
        
        speeds = np.zeros(4)
        for i in range(4):
            if abs_steps[i] > 0:
                speeds[i] = (abs_steps[i] / max_steps) * max_speed_rpm
        
        return speeds

    def get_home_position(self) -> np.ndarray:
        """Return the center of the workspace (home position)."""
        return np.array([
            self.config.frame_width / 2,
            self.config.frame_height / 2
        ])


# Convenience functions for standalone use

def load_config(config_path: str = "config.yaml") -> dict:
    """Load configuration from YAML file."""
    import yaml
    with open(config_path, 'r') as f:
        return yaml.safe_load(f)


def create_geometry(config_path: str = "config.yaml") -> CableGeometry:
    """Create CableGeometry from config file."""
    config = load_config(config_path)
    robot_config = RobotConfig.from_dict(config)
    return CableGeometry(robot_config)


if __name__ == "__main__":
    # Test kinematics
    import yaml
    
    config = load_config()
    robot_config = RobotConfig.from_dict(config)
    geom = CableGeometry(robot_config)
    
    print("=== Cable Robot Kinematics Test ===")
    print(f"Frame: {robot_config.frame_width}m x {robot_config.frame_height}m")
    print(f"Robot: {robot_config.robot_width}m x {robot_config.robot_height}m")
    print(f"Workspace: {geom.workspace_min} to {geom.workspace_max}")
    print()
    
    # Test positions
    test_positions = [
        geom.get_home_position(),  # Center
        np.array([0.5, 0.5]),
        np.array([1.0, 1.0]),
    ]
    
    for pos in test_positions:
        valid, msg = geom.validate_position(pos[0], pos[1])
        lengths = geom.position_to_cable_lengths(pos)
        steps = geom.cable_lengths_to_steps(lengths)
        recovered = geom.cable_lengths_to_position(lengths)
        
        print(f"Position: ({pos[0]:.3f}, {pos[1]:.3f})")
        print(f"  Valid: {valid} - {msg}")
        print(f"  Cable lengths: {lengths}")
        print(f"  Motor steps: {steps}")
        if recovered is not None:
            error = np.linalg.norm(pos - recovered)
            print(f"  Forward kinematics: ({recovered[0]:.3f}, {recovered[1]:.3f}), error: {error:.6f}m")
        print()
