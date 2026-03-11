"""
Motor controller for 3-cable window cleaning robot.

Handles position tracking, kinematics computation, and motor command generation.
Uses 3 cables: TL, TR, BL (indices 0, 1, 2).

Sends commands to a separate STM32 motor controller board via serial.
Command format: <motor_id>,<direction>,<rotations>,<speed_rpm>\n
"""

import math
import numpy as np
import serial
import time
from typing import Optional, Tuple
from dataclasses import dataclass

from kinematics import CableGeometry, RobotConfig, load_config


# Motor indices for 3-cable system
MOTOR_TL = 0  # Top-Left
MOTOR_TR = 1  # Top-Right
MOTOR_BL = 2  # Bottom-Left

NUM_CABLES = 3


@dataclass
class MotorCommand:
    """Command to send to STM32 motor controller."""
    motor_id: int
    direction: int      # 1 = lengthen cable (unwind), -1 = shorten (wind)
    rotations: float    # Number of motor rotations
    speed_rpm: float    # Motor speed in RPM
    
    def to_serial(self) -> str:
        """
        Convert to serial command string for STM32.
        Format: <motor_id>,<direction>,<rotations>,<speed_rpm>
        """
        return f"{self.motor_id},{self.direction},{self.rotations:.4f},{int(self.speed_rpm)}"


class MotorController:
    """
    Controls 3-cable robot positioning using kinematics.
    
    Tracks current position and generates coordinated motor commands
    for smooth motion to target positions. Communicates with STM32
    motor controller board via dedicated serial connection.
    """
    
    def __init__(self, config_path: str = "config.yaml"):
        """
        Initialize motor controller.
        
        Args:
            config_path: Path to YAML config file
        """
        self.config_dict = load_config(config_path)
        self.robot_config = RobotConfig.from_dict(self.config_dict)
        self.geometry = CableGeometry(self.robot_config)
        
        self.max_speed_rpm = self.config_dict.get('motor', {}).get('max_speed_rpm', 3000)
        self.steps_per_rev = self.config_dict.get('motor', {}).get('steps_per_rev', 3200)
        self.spool_radius = self.config_dict.get('motor', {}).get('spool_radius', 0.05)
        self.position_tolerance = self.config_dict.get('motion', {}).get('position_tolerance', 0.01)
        
        # Serial connection to motor STM32
        serial_config = self.config_dict.get('serial', {})
        self.motor_port = serial_config.get('motor_port', '/dev/ttyUSB0')
        self.baud_rate = serial_config.get('baud_rate', 115200)
        self.serial_timeout = serial_config.get('timeout', 1.0)
        self._serial = None
        
        # Current state
        self._position = self.geometry.get_home_position()
        self._cable_lengths = self.geometry.position_to_cable_lengths(self._position)[:NUM_CABLES]
        self._is_moving = False
        self._paused = False
    
    def connect(self) -> bool:
        """
        Connect to motor STM32 via serial.
        
        Returns True if connection successful.
        """
        try:
            self._serial = serial.Serial(
                port=self.motor_port,
                baudrate=self.baud_rate,
                timeout=self.serial_timeout
            )
            print(f"Connected to motor controller on {self.motor_port}")
            return True
        except Exception as e:
            print(f"Failed to connect to motor controller: {e}")
            self._serial = None
            return False
    
    def disconnect(self):
        """Disconnect from motor STM32."""
        if self._serial and self._serial.is_open:
            self._serial.close()
            self._serial = None
    
    @property
    def is_connected(self) -> bool:
        return self._serial is not None and self._serial.is_open
        
    @property
    def position(self) -> np.ndarray:
        """Current robot position (x, y)."""
        return self._position.copy()
    
    @property
    def cable_lengths(self) -> np.ndarray:
        """Current cable lengths for 3 cables."""
        return self._cable_lengths.copy()
    
    @property
    def is_moving(self) -> bool:
        return self._is_moving
    
    def set_position(self, x: float, y: float) -> bool:
        """
        Manually set current position (for calibration).
        
        Returns True if position is valid.
        """
        valid, msg = self.geometry.validate_position(x, y)
        if not valid:
            print(f"Invalid position: {msg}")
            return False
        
        self._position = np.array([x, y])
        self._cable_lengths = self.geometry.position_to_cable_lengths(self._position)[:NUM_CABLES]
        return True
    
    def validate_target(self, x: float, y: float) -> Tuple[bool, str]:
        """Validate a target position."""
        return self.geometry.validate_position(x, y)
    
    def compute_move(self, target_x: float, target_y: float) -> Optional[list]:
        """
        Compute motor commands to move to target position.
        
        Calculates cable length deltas from kinematics, converts to motor
        rotations, and generates coordinated commands for the STM32.
        
        Returns:
            List of MotorCommand objects, or None if invalid target
        """
        valid, msg = self.validate_target(target_x, target_y)
        if not valid:
            print(f"Invalid target: {msg}")
            return None
        
        target = np.array([target_x, target_y])
        target_lengths = self.geometry.position_to_cable_lengths(target)[:NUM_CABLES]
        
        # Compute delta lengths (meters)
        delta_lengths = target_lengths - self._cable_lengths
        
        # Convert delta lengths to rotations
        # rotations = delta_length / (2 * pi * spool_radius)
        spool_circumference = 2 * math.pi * self.spool_radius
        delta_rotations = delta_lengths / spool_circumference
        
        # Compute coordinated speeds so all motors finish together
        abs_rotations = np.abs(delta_rotations)
        max_rotations = np.max(abs_rotations)
        
        if max_rotations < 0.001:  # Less than 1/1000 of a rotation - already at target
            return []
        
        commands = []
        for i in range(NUM_CABLES):
            if abs_rotations[i] > 0.001:
                # Scale speed proportionally so all motors finish at same time
                speed = (abs_rotations[i] / max_rotations) * self.max_speed_rpm
                # Direction: positive delta = lengthen cable = 1, negative = shorten = -1
                direction = 1 if delta_rotations[i] >= 0 else -1
                commands.append(MotorCommand(
                    motor_id=i,
                    direction=direction,
                    rotations=abs(delta_rotations[i]),
                    speed_rpm=speed
                ))
        
        return commands
    
    def goto(self, x: float, y: float) -> bool:
        """
        Move robot to target position.
        
        Returns True if move was initiated successfully.
        """
        if self._paused:
            print("Cannot move: paused")
            return False
            
        commands = self.compute_move(x, y)
        if commands is None:
            return False
        
        if len(commands) == 0:
            print(f"Already at position ({x:.3f}, {y:.3f})")
            return True
        
        # Send commands
        self._is_moving = True
        for cmd in commands:
            self._send_command(cmd)
        
        # Update internal state (assume move completes)
        target = np.array([x, y])
        self._position = target
        self._cable_lengths = self.geometry.position_to_cable_lengths(target)[:NUM_CABLES]
        self._is_moving = False
        
        return True
    
    def move_relative(self, dx: float, dy: float) -> bool:
        """Move by relative offset."""
        target_x = self._position[0] + dx
        target_y = self._position[1] + dy
        return self.goto(target_x, target_y)
    
    def move_direction(self, direction: str, distance: float = 0.05) -> bool:
        """
        Move in cardinal direction.
        
        Args:
            direction: 'up', 'down', 'left', 'right'
            distance: Distance to move in meters
        """
        deltas = {
            'up': (0, distance),
            'down': (0, -distance),
            'left': (-distance, 0),
            'right': (distance, 0),
        }
        
        if direction not in deltas:
            print(f"Invalid direction: {direction}")
            return False
        
        dx, dy = deltas[direction]
        return self.move_relative(dx, dy)
    
    def home(self) -> bool:
        """Return to home position (center of workspace)."""
        home_pos = self.geometry.get_home_position()
        return self.goto(home_pos[0], home_pos[1])
    
    def stop(self):
        """Emergency stop - halt all motors."""
        self._is_moving = False
        self._send_raw("STOP")
    
    def pause(self):
        """Pause motion."""
        self._paused = True
        self.stop()
    
    def resume(self):
        """Resume motion."""
        self._paused = False
    
    def get_status(self) -> dict:
        """Get current controller status."""
        return {
            'position': {'x': float(self._position[0]), 'y': float(self._position[1])},
            'cable_lengths': self._cable_lengths.tolist(),
            'is_moving': self._is_moving,
            'paused': self._paused,
            'connected': self.is_connected,
        }
    
    def _send_command(self, cmd: MotorCommand):
        """Send motor command via serial to STM32."""
        msg = cmd.to_serial()
        print(f"Motor cmd: {msg}")
        self._send_raw(msg)
    
    def _send_raw(self, message: str):
        """Send raw message to motor STM32 via serial."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.write((message + '\n').encode('utf-8'))
            except Exception as e:
                print(f"Serial write error: {e}")
