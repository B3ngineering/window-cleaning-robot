"""
Motor controller for 4-motor cable window cleaning robot.

Handles position tracking, kinematics computation, and motor command generation.
Uses 3 active cables: TL, TR, BL. Motor 2 (BR) reserved.

Internal motor indices:
    BL=0, TR=1, BR=2 (reserved), TL=3

STM32 motor IDs:
    BR=1, TL=2, BL=3, TR=4

Serial packet format:
    [count, motor_id, dir, speed_hi, speed_lo, pulses_hi, pulses_lo, ...]
where speed is encoded as steps/sec // 10 and pulses are step counts.
"""

import math
import numpy as np
import serial
import time
from typing import Optional, Tuple, List
from dataclasses import dataclass, field

from kinematics import CableGeometry, RobotConfig, load_config


# Motor indices (matching serial code)
MOTOR_BL = 0  # Bottom-Left
MOTOR_TR = 3  # Top-Right
MOTOR_BR = 2  # Bottom-Right (reserved, sends zeros)
MOTOR_TL = 1  # Top-Left

NUM_CABLES = 3   # Active cables for kinematics
NUM_MOTORS = 4   # Total motors in command structure

# Active motors and their corresponding geometry/cable indices
# Geometry order from kinematics: [TL=0, TR=1, BL=2, BR=3]
ACTIVE_MOTORS = [MOTOR_BL, MOTOR_TR, MOTOR_TL]  # Motors 0, 1, 3
MOTOR_TO_CABLE = [2, 1, None, 0]  # Motor index -> cable/geometry index (None = reserved)

# Direction encoding used internally by the controller.
# The top motors (TL/TR) match this encoding on the wire, while the
# bottom motors (BL/BR) are wired in reverse and must be flipped before send.
DIR_CCW = 0  # Counter-clockwise: lengthen cable (unwind)
DIR_CW = 1   # Clockwise: shorten cable (wind)

# STM32 board motor IDs from sample control code
STM_MOTOR_IDS = {
    MOTOR_BR: 1,
    MOTOR_TL: 2,
    MOTOR_BL: 3,
    MOTOR_TR: 4,
}


@dataclass
class MoveCommand:
    """
    Unified command for all 4 motors.
    
    Each motor has [direction, rotations, speed_rpm]:
        - direction: 0 = CCW (lengthen cable / unwind), 1 = CW (shorten cable / wind)
            (no movement is represented by rotations=0 and/or speed_rpm=0)
    - rotations: Number of motor rotations (absolute value)
    - speed_rpm: Motor speed in RPM
    """
    motors: List[List] = field(default_factory=lambda: [[0, 0.0, 0] for _ in range(NUM_MOTORS)])
    
    def to_debug_string(self) -> str:
        """
        Convert to readable command string for logging.
        Format: [[dir,rot,spd],[dir,rot,spd],[dir,rot,spd],[dir,rot,spd]]
        """
        motor_strs = []
        for m in self.motors:
            motor_strs.append(f"[{m[0]},{m[1]:.4f},{int(m[2])}]")
        return "[" + ",".join(motor_strs) + "]"


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
            time.sleep(0.1)
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
    
    def compute_move(self, target_x: float, target_y: float) -> Optional[MoveCommand]:
        """
        Compute motor command to move to target position.
        
        Calculates cable length deltas from kinematics, converts to motor
        rotations, and generates a unified command for all 4 motors.
        
        Returns:
            MoveCommand with all 4 motors, or None if invalid/already at target
        """
        valid, msg = self.validate_target(target_x, target_y)
        if not valid:
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
            return None
        
        # Build command for all 4 motors
        # Map geometry/cable indices to motor indices
        command = MoveCommand()
        for motor_idx in ACTIVE_MOTORS:
            cable_idx = MOTOR_TO_CABLE[motor_idx]
            if abs_rotations[cable_idx] > 0.001:
                # Scale speed proportionally so all motors finish at same time
                speed = (abs_rotations[cable_idx] / max_rotations) * self.max_speed_rpm
                # Direction: positive delta = lengthen cable (CCW=0), negative = shorten (CW=1)
                direction = DIR_CCW if delta_rotations[cable_idx] >= 0 else DIR_CW
                command.motors[motor_idx] = [direction, abs(delta_rotations[cable_idx]), speed]
        
        # Motor 2 (BR) stays as zeros - reserved for future use
        
        return command
    
    def goto(self, x: float, y: float) -> bool:
        """
        Move robot to target position.
        
        Returns True if move was initiated successfully.
        """
        if self._paused:
            print("Cannot move: paused")
            return False
        
        # Validate target first
        valid, msg = self.validate_target(x, y)
        if not valid:
            print(f"Invalid target: {msg}")
            return False
            
        command = self.compute_move(x, y)
        if command is None:
            # compute_move returns None if already at target (max_rotations < 0.001)
            print(f"Already at position ({x:.3f}, {y:.3f})")
            return True

        target = np.array([x, y])
        target_cable_lengths = self.geometry.position_to_cable_lengths(target)[:NUM_CABLES]
        delta_lengths = target_cable_lengths - self._cable_lengths

        cable_names = ["TL", "TR", "BL"]
        print("Cable move plan:")
        print(f"  {'Cable':<6} {'Current (m)':>12} {'Target (m)':>12} {'Delta (m)':>10} {'Direction'}")
        print(f"  {'-'*6} {'-'*12} {'-'*12} {'-'*10} {'-'*12}")
        for i, name in enumerate(cable_names):
            direction_label = "unwind (CCW)" if delta_lengths[i] >= 0 else "wind   (CW) "
            print(
                f"  {name:<6} {self._cable_lengths[i]:>12.4f} "
                f"{target_cable_lengths[i]:>12.4f} "
                f"{delta_lengths[i]:>+10.4f} {direction_label}"
            )
        
        # Send unified command for all 4 motors
        self._is_moving = True
        self._send_command(command)
        
        # Update internal state (assume move completes)
        self._position = target
        self._cable_lengths = target_cable_lengths
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
        self._send_emergency_stop_packet()
    
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
    
    def _send_command(self, cmd: MoveCommand):
        """Send motor command via serial to STM32."""
        packet = self._build_packet(cmd)
        
        print(f"Motor cmd: {cmd.to_debug_string()}")
        print(f"Motor packet: {list(packet)}")
        self._send_raw(packet)

    def _send_emergency_stop_packet(self):
        """Send explicit zero-speed stop command for all motors."""
        commands = []
        for motor_idx, stm_motor_id in STM_MOTOR_IDS.items():
            commands.append((stm_motor_id, self._encode_motor_direction(motor_idx, DIR_CCW), 0, 0))

        packet = bytearray([len(commands)])
        for motor_id, direction, steps_per_sec, pulses in commands:
            speed_enc = steps_per_sec // 10
            packet.extend([
                motor_id,
                direction,
                (speed_enc >> 8) & 0xFF,
                speed_enc & 0xFF,
                (pulses >> 8) & 0xFF,
                pulses & 0xFF,
            ])

        print("Emergency stop: zero-speed command to all motors")
        print(f"Motor packet: {list(packet)}")
        self._send_raw(bytes(packet))
    
    def _build_packet(self, cmd: MoveCommand) -> bytes:
        """Build STM32 packet matching the expected binary wire format."""
        commands = []

        for motor_idx in ACTIVE_MOTORS:
            direction, rotations, speed_rpm = cmd.motors[motor_idx]
            if rotations <= 0 or speed_rpm <= 0:
                continue
            if direction not in (DIR_CW, DIR_CCW):
                continue

            stm_motor_id = STM_MOTOR_IDS[motor_idx]
            stm_direction = self._encode_motor_direction(motor_idx, direction)
            steps_per_sec = max(1, int((float(speed_rpm) * self.steps_per_rev) / 60.0))
            pulses = max(1, int(round(float(rotations) * self.steps_per_rev)))
            steps_per_sec = min(0xFFFF, steps_per_sec)
            pulses = min(0xFFFF, pulses)

            # commands: [motor_id, dir, steps_per_sec, pulses]
            commands.append((stm_motor_id, stm_direction, steps_per_sec, pulses))

        packet = bytearray([len(commands)])
        for motor_id, direction, steps_per_sec, pulses in commands:
            speed_enc = steps_per_sec // 10
            packet.extend([
                motor_id,
                direction,
                (speed_enc >> 8) & 0xFF,
                speed_enc & 0xFF,
                (pulses >> 8) & 0xFF,
                pulses & 0xFF,
            ])

        return bytes(packet)

    def _encode_motor_direction(self, motor_idx: int, direction: int) -> int:
        """Convert controller direction to the motor's wiring-specific wire value."""
        if motor_idx in (MOTOR_BL, MOTOR_BR):
            return DIR_CW if direction == DIR_CCW else DIR_CCW
        return direction

    def _send_raw(self, packet: bytes):
        """Send raw binary packet to motor STM32 via serial."""
        if self._serial and self._serial.is_open:
            try:
                self._serial.write(packet)
            except Exception as e:
                print(f"Serial write error: {e}") 