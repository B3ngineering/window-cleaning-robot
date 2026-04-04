"""
Cable robot kinematics for window cleaning robot.

Provides forward and inverse kinematics for a 4-cable parallel robot
with dynamic cable attachments that slide on top/bottom semicircular rails.

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
    top_semicircle_radius: float
    bottom_semicircle_radius: float
    spool_radius: float
    steps_per_rev: int
    safety_margin: float = 0.1

    @classmethod
    def from_dict(cls, config: dict) -> 'RobotConfig':
        """Create RobotConfig from config dictionary."""
        robot_cfg = config['robot']
        default_radius = robot_cfg['width'] / 2.0
        return cls(
            frame_width=config['frame']['width'],
            frame_height=config['frame']['height'],
            robot_width=robot_cfg['width'],
            robot_height=robot_cfg['height'],
            top_semicircle_radius=robot_cfg.get('top_semicircle_radius', default_radius),
            bottom_semicircle_radius=robot_cfg.get('bottom_semicircle_radius', default_radius),
            spool_radius=config['motor']['spool_radius'],
            steps_per_rev=config['motor']['steps_per_rev'],
            safety_margin=config.get('motion', {}).get('safety_margin', 0.1)
        )


class CableGeometry:
    """
    Cable kinematics for 4-cable parallel robot.
    
    Top cables (TL/TR) attach to a top semicircular rail, and bottom cables
    (BL/BR) attach to a bottom semicircular rail. Each attachment point slides
    along its semicircle to align with the cable direction, preserving robot pose.
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

        # Semicircle centers relative to robot center
        self.top_semicircle_center_offset = np.array([0.0, config.robot_height / 2.0])
        self.bottom_semicircle_center_offset = np.array([0.0, -config.robot_height / 2.0])

        # Keep left cables on left half and right cables on right half.
        # Geometry order: [TL, TR, BL, BR]
        self._attachment_side_sign = np.array([-1.0, 1.0, -1.0, 1.0])

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

    def _project_direction_to_semicircle(self, direction: np.ndarray, is_top: bool,
                                         side_sign: float) -> np.ndarray:
        """Project a direction onto the allowed semicircle sector."""
        d = np.array(direction, dtype=float)
        norm = np.linalg.norm(d)
        if norm < 1e-9:
            d = np.array([side_sign, 0.0], dtype=float)
        else:
            d = d / norm

        # Top rail: y >= 0, Bottom rail: y <= 0
        if is_top and d[1] < 0:
            d[1] = 0.0
        if (not is_top) and d[1] > 0:
            d[1] = 0.0

        # Left cable stays on x <= 0, right cable stays on x >= 0
        if side_sign < 0 and d[0] > 0:
            d[0] = 0.0
        if side_sign > 0 and d[0] < 0:
            d[0] = 0.0

        norm = np.linalg.norm(d)
        if norm < 1e-9:
            return np.array([side_sign, 0.0], dtype=float)
        return d / norm

    def get_attachment_point(self, robot_center: np.ndarray, index: int) -> np.ndarray:
        """Get dynamic world attachment point for cable index [TL, TR, BL, BR]."""
        is_top = index in (MOTOR_TL, MOTOR_TR)
        side_sign = self._attachment_side_sign[index]

        if is_top:
            rail_center = robot_center + self.top_semicircle_center_offset
            radius = self.config.top_semicircle_radius
        else:
            rail_center = robot_center + self.bottom_semicircle_center_offset
            radius = self.config.bottom_semicircle_radius

        # Frictionless slider on rail tends to align cable with local radius.
        desired_dir = self.anchors[index] - rail_center
        rail_dir = self._project_direction_to_semicircle(desired_dir, is_top, side_sign)
        return rail_center + radius * rail_dir

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
        if len(lengths) != 4:
            return None

        # Nonlinear least-squares solve (Gauss-Newton) because attachment points
        # move with position on semicircular rails.
        position = self.get_home_position().astype(float)
        step = 1e-4

        for _ in range(30):
            predicted = self.position_to_cable_lengths(position)
            residual = predicted - lengths

            if np.linalg.norm(residual) < 1e-6:
                break

            jacobian = np.zeros((4, 2), dtype=float)
            for axis in range(2):
                offset = np.zeros(2, dtype=float)
                offset[axis] = step
                plus = self.position_to_cable_lengths(position + offset)
                minus = self.position_to_cable_lengths(position - offset)
                jacobian[:, axis] = (plus - minus) / (2.0 * step)

            try:
                delta, *_ = np.linalg.lstsq(jacobian, -residual, rcond=None)
            except np.linalg.LinAlgError:
                return None

            position = position + delta

            # Keep iterate in workspace neighborhood for stable convergence.
            position[0] = float(np.clip(position[0], self.workspace_min[0], self.workspace_max[0]))
            position[1] = float(np.clip(position[1], self.workspace_min[1], self.workspace_max[1]))

            if np.linalg.norm(delta) < 1e-7:
                break

        final_error = np.linalg.norm(self.position_to_cable_lengths(position) - lengths)
        if final_error > 5e-3:
            return None
        return position

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