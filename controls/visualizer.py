#!/usr/bin/env python3
"""
Real-time matplotlib visualizer for window cleaning robot.

Shows the frame, anchors, cables, and robot platform. Receives commands
via Unix socket and animates robot movement in real-time.

Can run in two modes:
  - Standalone: Only simulates commands locally
  - Proxy: Forwards commands to real robot after visualization

Usage:
    python visualizer.py                 # Standalone mode
    python visualizer.py --proxy         # Proxy mode (forwards to real system)
"""

import os
import sys
import math
import socket
import threading
import queue
import time
import argparse
import random
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from matplotlib.animation import FuncAnimation
from dataclasses import dataclass, field
from typing import Optional, List, Tuple

from kinematics import CableGeometry, RobotConfig, load_config

# Socket paths
VISUALIZER_SOCKET_PATH = "/tmp/window_robot_viz.sock"
REAL_ROBOT_SOCKET_PATH = "/tmp/window_robot_cmd.sock"
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")

# Motor indices (matching serial code from motor_controller.py)
MOTOR_BL = 0  # Bottom-Left
MOTOR_TR = 1  # Top-Right
MOTOR_BR = 2  # Bottom-Right (reserved)
MOTOR_TL = 3  # Top-Left

# Geometry order from kinematics: [TL=0, TR=1, BL=2, BR=3]
MOTOR_TO_CABLE = [2, 1, None, 0]  # Motor index -> cable/geometry index
CABLE_TO_MOTOR = {0: 3, 1: 1, 2: 0}  # Cable/geometry index -> motor index (TL->M3, TR->M1, BL->M0)
ACTIVE_MOTORS = [0, 1, 3]  # BL, TR, TL
NUM_MOTORS = 4
NUM_CABLES = 3

# STM32 direction byte values (mirrors motor_controller.py)
DIR_CCW = 0  # Counter-clockwise (lengthens cable / unwinds)
DIR_CW  = 1  # Clockwise  (shortens cable / winds)


@dataclass
class MotorCommand:
    """
    Motor command matching serial format.
    
    Each motor has [direction, rotations, speed_rpm]:
        - direction: 0 = CCW (lengthen cable / unwind), 1 = CW (shorten cable / wind)
            (no movement is represented by rotations=0 and/or speed_rpm=0)
        - rotations: Number of motor rotations (absolute)
        STM32 wire encoding: CCW = 0, CW = 1 (see DIR_CCW / DIR_CW constants)
    - speed_rpm: Motor speed in RPM
    """
    motors: List[List] = field(default_factory=lambda: [[0, 0.0, 0] for _ in range(NUM_MOTORS)])
    
    def to_serial(self) -> str:
        """Convert to serial command string for STM32."""
        motor_strs = []
        for m in self.motors:
            motor_strs.append(f"[{m[0]},{m[1]:.4f},{int(m[2])}]")
        return "[" + ",".join(motor_strs) + "]"


@dataclass
class SimState:
    """Simulated robot state with motor-command-driven physics."""
    position: np.ndarray
    cable_lengths: np.ndarray  # Current lengths [TL, TR, BL] in geometry order
    target_position: Optional[np.ndarray] = None
    is_moving: bool = False
    move_start_time: float = 0.0
    move_duration: float = 0.0
    # Motor command state (the actual command that would be sent to STM32)
    motor_command: Optional[MotorCommand] = None
    start_cable_lengths: Optional[np.ndarray] = None  # Starting lengths for animation
    rotations_applied: Optional[np.ndarray] = None  # Track rotations applied per motor


class RobotVisualizer:
    """
    Real-time matplotlib visualizer for cable robot.
    
    Displays frame, anchors, cables, robot platform and animates
    movement based on received commands.
    """
    
    def __init__(self, config_path: str = CONFIG_PATH, proxy_mode: bool = False):
        """Initialize visualizer with config."""
        self.config_dict = load_config(config_path)
        self.robot_config = RobotConfig.from_dict(self.config_dict)
        self.geometry = CableGeometry(self.robot_config)
        self.proxy_mode = proxy_mode
        
        # Motor/motion parameters
        self.max_speed_rpm = self.config_dict.get('motor', {}).get('max_speed_rpm', 3000)
        self.spool_radius = self.config_dict.get('motor', {}).get('spool_radius', 0.025)
        
        # Command queue for thread-safe communication
        self.command_queue = queue.Queue()
        self.stop_event = threading.Event()
        
        # Initialize state at home position
        home_pos = self.geometry.get_home_position()
        self.state = SimState(
            position=home_pos.copy(),
            cable_lengths=self.geometry.position_to_cable_lengths(home_pos)[:NUM_CABLES]
        )

        # Clean routine simulation state
        self.clean_max_attempts = 3
        self.clean_success_probability = self.config_dict.get('simulation', {}).get('clean_success_probability', 0.6)
        self.clean_session = None
        
        # Setup matplotlib figure
        self._setup_plot()
    
    def _setup_plot(self):
        """Setup matplotlib figure and artists."""
        plt.style.use('dark_background')
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.fig.canvas.manager.set_window_title('Window Cleaning Robot Visualizer')
        
        fw = self.robot_config.frame_width
        fh = self.robot_config.frame_height
        rw = self.robot_config.robot_width
        rh = self.robot_config.robot_height
        
        # Set axis limits with padding
        pad = 0.2
        self.ax.set_xlim(-pad, fw + pad)
        self.ax.set_ylim(-pad, fh + pad)
        self.ax.set_aspect('equal')
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        self.ax.set_title('Window Cleaning Robot Simulator')
        self.ax.grid(True, alpha=0.3)
        
        # Draw frame rectangle
        frame_rect = patches.Rectangle(
            (0, 0), fw, fh,
            linewidth=2, edgecolor='white', facecolor='none', linestyle='--'
        )
        self.ax.add_patch(frame_rect)
        
        # Draw workspace boundary (safety margin)
        ws_min = self.geometry.workspace_min
        ws_max = self.geometry.workspace_max
        workspace_rect = patches.Rectangle(
            (ws_min[0], ws_min[1]),
            ws_max[0] - ws_min[0],
            ws_max[1] - ws_min[1],
            linewidth=1, edgecolor='green', facecolor='none', linestyle=':'
        )
        self.ax.add_patch(workspace_rect)
        
        # Draw anchors (corners)
        anchor_colors = ['cyan', 'magenta', 'yellow', 'gray']  # TL, TR, BL, BR
        anchor_labels = ['TL (M3)', 'TR (M1)', 'BL (M0)', 'BR (M2)']
        for i, (anchor, color, label) in enumerate(zip(self.geometry.anchors, anchor_colors, anchor_labels)):
            self.ax.plot(anchor[0], anchor[1], 'o', color=color, markersize=12)
            offset = [0.05, 0.05]
            if anchor[0] > fw / 2:
                offset[0] = -0.15
            if anchor[1] < fh / 2:
                offset[1] = -0.1
            self.ax.annotate(label, anchor, xytext=(anchor[0] + offset[0], anchor[1] + offset[1]),
                           fontsize=9, color=color)
        
        # Initialize cable lines (TL, TR, BL - active cables)
        self.cable_lines = []
        cable_colors = ['cyan', 'magenta', 'yellow']
        for i in range(NUM_CABLES):
            line, = self.ax.plot([], [], '-', color=cable_colors[i], linewidth=1.5, alpha=0.7)
            self.cable_lines.append(line)
        
        # Initialize robot platform rectangle
        pos = self.state.position
        self.robot_patch = patches.Rectangle(
            (pos[0] - rw/2, pos[1] - rh/2), rw, rh,
            linewidth=2, edgecolor='lime', facecolor='darkgreen', alpha=0.8
        )
        self.ax.add_patch(self.robot_patch)
        
        # Robot center marker
        self.robot_center, = self.ax.plot(pos[0], pos[1], 'o', color='white', markersize=6)
        
        # Target marker (hidden initially)
        self.target_marker, = self.ax.plot([], [], 'x', color='red', markersize=15, mew=3)
        
        # Status text
        self.status_text = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            fontsize=10, verticalalignment='top', fontfamily='monospace',
            color='white', bbox=dict(boxstyle='round', facecolor='black', alpha=0.7)
        )
        
        # Command info text
        mode_str = "PROXY MODE" if self.proxy_mode else "STANDALONE MODE"
        self.mode_text = self.ax.text(
            0.98, 0.98, mode_str, transform=self.ax.transAxes,
            fontsize=10, verticalalignment='top', horizontalalignment='right',
            color='lime' if not self.proxy_mode else 'orange',
            bbox=dict(boxstyle='round', facecolor='black', alpha=0.7)
        )
    
    def _update_visuals(self):
        """Update all visual elements based on current state."""
        pos = self.state.position
        rw = self.robot_config.robot_width
        rh = self.robot_config.robot_height
        
        # Update robot platform position
        self.robot_patch.set_xy((pos[0] - rw/2, pos[1] - rh/2))
        self.robot_center.set_data([pos[0]], [pos[1]])
        
        # Update cables from anchors to robot attachment points
        # Geometry order: [TL=0, TR=1, BL=2], anchors same order
        for i in range(NUM_CABLES):
            anchor = self.geometry.anchors[i]
            attachment = self.geometry.get_attachment_point(pos, i)
            self.cable_lines[i].set_data(
                [anchor[0], attachment[0]],
                [anchor[1], attachment[1]]
            )
        
        # Update target marker
        if self.state.target_position is not None and self.state.is_moving:
            self.target_marker.set_data([self.state.target_position[0]], [self.state.target_position[1]])
        else:
            self.target_marker.set_data([], [])
        
        # Update status text
        cable_lengths = self.geometry.position_to_cable_lengths(pos)[:NUM_CABLES]
        clean_status = "IDLE"
        if self.clean_session and self.clean_session.get("active"):
            clean_status = (
                f"{self.clean_session['phase']} "
                f"({self.clean_session['attempt']}/{self.clean_session['max_attempts']})"
            )

        status = (
            f"Position: ({pos[0]:.3f}, {pos[1]:.3f})\n"
            f"Cables: TL={cable_lengths[0]:.3f} TR={cable_lengths[1]:.3f} BL={cable_lengths[2]:.3f}\n"
            f"Status: {'MOVING' if self.state.is_moving else 'IDLE'}\n"
            f"Clean: {clean_status}"
        )
        if self.state.target_position is not None and self.state.is_moving:
            status += f"\nTarget: ({self.state.target_position[0]:.3f}, {self.state.target_position[1]:.3f})"
        self.status_text.set_text(status)
    
    def _animate(self, frame):
        """Animation update function called by FuncAnimation."""
        # Process any pending commands
        while not self.command_queue.empty():
            try:
                cmd = self.command_queue.get_nowait()
                self._process_command(cmd)
            except queue.Empty:
                break
        
        # Update motion simulation using motor commands
        if self.state.is_moving and self.state.motor_command is not None:
            elapsed = time.time() - self.state.move_start_time
            spool_circumference = 2 * math.pi * self.spool_radius
            
            # Apply motor commands to cable lengths
            # For each active motor, compute rotations based on speed and elapsed time
            move_complete = True
            
            for motor_idx in ACTIVE_MOTORS:
                motor_cmd = self.state.motor_command.motors[motor_idx]
                direction = motor_cmd[0]  # DIR_CCW=0 (lengthen), DIR_CW=1 (shorten)
                target_rotations = motor_cmd[1]  # Total rotations to apply
                speed_rpm = motor_cmd[2]  # Motor speed in RPM
                
                if target_rotations < 0.001 or speed_rpm < 1:
                    continue  # No movement for this motor
                
                # Compute rotations based on elapsed time
                # rotations = speed_rpm / 60 * elapsed_seconds
                rotations_done = (speed_rpm / 60.0) * elapsed
                rotations_done = min(rotations_done, target_rotations)  # Clamp to target
                
                if rotations_done < target_rotations:
                    move_complete = False
                
                # Convert rotations to cable length change
                # Convert direction code to sign for cable delta
                direction_sign = 1.0 if direction == DIR_CCW else -1.0
                # delta_length = direction_sign * rotations * spool_circumference
                cable_idx = MOTOR_TO_CABLE[motor_idx]
                delta_length = direction_sign * rotations_done * spool_circumference
                self.state.cable_lengths[cable_idx] = self.state.start_cable_lengths[cable_idx] + delta_length
            
            # Use forward kinematics to compute position from cable lengths
            full_lengths = np.zeros(4)
            full_lengths[:NUM_CABLES] = self.state.cable_lengths
            # BR cable not used in trilateration
            full_lengths[3] = self.geometry.position_to_cable_lengths(self.state.position)[3]
            
            new_position = self.geometry.cable_lengths_to_position(full_lengths)
            if new_position is not None:
                self.state.position = new_position
            
            # Check if move is complete
            if move_complete or elapsed >= self.state.move_duration:
                # Final position from forward kinematics (tests motor command accuracy)
                final_lengths = np.zeros(4)
                final_lengths[:NUM_CABLES] = self.state.cable_lengths
                final_lengths[3] = 0  # Not used
                final_pos = self.geometry.cable_lengths_to_position(final_lengths)
                
                if final_pos is not None:
                    self.state.position = final_pos
                    # Check accuracy: did we reach the target?
                    error = np.linalg.norm(final_pos - self.state.target_position)
                    if error > 0.001:
                        print(f"Position error: {error:.4f}m (target: {self.state.target_position}, actual: {final_pos})")
                
                self.state.is_moving = False
                self.state.motor_command = None

                if self.clean_session and self.clean_session.get("active"):
                    self._advance_clean_session_after_move()
        
        self._update_visuals()
        return [self.robot_patch, self.robot_center, self.target_marker, 
                self.status_text] + self.cable_lines
    
    def _process_command(self, cmd_str: str):
        """Process a command string and update simulation state."""
        cmd = cmd_str.lower().split()
        if not cmd:
            return
        
        action = cmd[0]
        args = cmd[1:]
        
        if action == "goto":
            if len(args) >= 2:
                try:
                    x, y = float(args[0]), float(args[1])
                    self._start_move_to(x, y)
                except ValueError:
                    print(f"Invalid goto args: {args}")
        
        elif action == "move":
            if len(args) >= 1:
                direction = args[0]
                distance = float(args[1]) if len(args) > 1 else 0.05
                deltas = {
                    'up': (0, distance),
                    'down': (0, -distance),
                    'left': (-distance, 0),
                    'right': (distance, 0),
                }
                if direction in deltas:
                    dx, dy = deltas[direction]
                    target_x = self.state.position[0] + dx
                    target_y = self.state.position[1] + dy
                    self._start_move_to(target_x, target_y)
        
        elif action == "home":
            home_pos = self.geometry.get_home_position()
            self._start_move_to(home_pos[0], home_pos[1])

        elif action == "clean":
            self._start_clean_session()
        
        elif action == "stop":
            self.state.is_moving = False
            self.state.target_position = None
            if self.clean_session and self.clean_session.get("active"):
                self.clean_session["active"] = False
                print("Clean routine stopped")
        
        elif action == "setpos":
            if len(args) >= 2:
                try:
                    x, y = float(args[0]), float(args[1])
                    valid, _ = self.geometry.validate_position(x, y)
                    if valid:
                        self.state.position = np.array([x, y])
                        self.state.cable_lengths = self.geometry.position_to_cable_lengths(self.state.position)[:NUM_CABLES]
                except ValueError:
                    pass

    def _simulate_clean_result(self) -> bool:
        """Return True when clean is detected, False when dirty is detected."""
        return random.random() < self.clean_success_probability

    def _start_clean_session(self):
        """Start CLEANING->CHECKING simulated loop with retries."""
        if self.clean_session and self.clean_session.get("active"):
            print("Clean routine already running")
            return

        current_x = float(self.state.position[0])
        self.clean_session = {
            "active": True,
            "phase": "CLEANING",
            "attempt": 1,
            "max_attempts": self.clean_max_attempts,
            "x": current_x,
        }

        bottom_y = float(self.geometry.workspace_min[1])
        print(f"[clean] Attempt 1/{self.clean_max_attempts}: CLEANING down to y={bottom_y:.3f}")
        self._start_move_to(current_x, bottom_y)

    def _advance_clean_session_after_move(self):
        """Advance clean routine after each completed move."""
        if not self.clean_session or not self.clean_session.get("active"):
            return

        phase = self.clean_session.get("phase")
        x = float(self.clean_session.get("x", self.state.position[0]))

        if phase == "CLEANING":
            self.clean_session["phase"] = "CHECKING"
            top_y = float(self.geometry.workspace_max[1])
            print(f"[clean] CHECKING up to y={top_y:.3f}")
            self._start_move_to(x, top_y)
            return

        if phase == "CHECKING":
            clean_detected = self._simulate_clean_result()
            print(f"[clean] CHECKING result: {'clean' if clean_detected else 'dirty'}")

            if clean_detected:
                self.clean_session["active"] = False
                self.clean_session["phase"] = "IDLE"
                print("[clean] Routine completed: window confirmed clean")
                return

            if self.clean_session["attempt"] >= self.clean_session["max_attempts"]:
                self.clean_session["active"] = False
                self.clean_session["phase"] = "IDLE"
                print("[clean] Routine ended without clean confirmation")
                return

            self.clean_session["attempt"] += 1
            self.clean_session["phase"] = "CLEANING"
            self.clean_session["x"] = float(self.state.position[0])
            bottom_y = float(self.geometry.workspace_min[1])
            print(
                f"[clean] Attempt {self.clean_session['attempt']}/{self.clean_session['max_attempts']}: "
                f"CLEANING down to y={bottom_y:.3f}"
            )
            self._start_move_to(self.clean_session["x"], bottom_y)
    
    def _compute_motor_command(self, target_x: float, target_y: float) -> Optional[MotorCommand]:
        """
        Compute motor command to move to target position.
        
        This replicates motor_controller.compute_move() logic exactly
        to test that motor commands produce correct movement.
        """
        valid, msg = self.geometry.validate_position(target_x, target_y)
        if not valid:
            return None
        
        target = np.array([target_x, target_y])
        target_lengths = self.geometry.position_to_cable_lengths(target)[:NUM_CABLES]
        current_lengths = self.state.cable_lengths.copy()
        
        # Compute delta lengths (meters)
        delta_lengths = target_lengths - current_lengths
        
        # Convert delta lengths to rotations
        spool_circumference = 2 * math.pi * self.spool_radius
        delta_rotations = delta_lengths / spool_circumference
        
        # Compute coordinated speeds so all motors finish together
        abs_rotations = np.abs(delta_rotations)
        max_rotations = np.max(abs_rotations)
        
        if max_rotations < 0.001:  # Already at target
            return None
        
        # Build motor command (same logic as motor_controller.compute_move)
        command = MotorCommand()
        for motor_idx in ACTIVE_MOTORS:
            cable_idx = MOTOR_TO_CABLE[motor_idx]
            if abs_rotations[cable_idx] > 0.001:
                # Scale speed proportionally so all motors finish at same time
                speed = (abs_rotations[cable_idx] / max_rotations) * self.max_speed_rpm
                # Direction: positive delta = lengthen cable (CCW=0), negative = shorten (CW=1)
                direction = DIR_CCW if delta_rotations[cable_idx] >= 0 else DIR_CW
                command.motors[motor_idx] = [direction, abs(delta_rotations[cable_idx]), speed]
        
        # Motor 2 (BR) stays as zeros - reserved
        return command
    
    def _start_move_to(self, x: float, y: float):
        """Start animated move to target position using motor commands."""
        valid, msg = self.geometry.validate_position(x, y)
        if not valid:
            print(f"Invalid target ({x:.3f}, {y:.3f}): {msg}")
            return
        
        target = np.array([x, y])
        
        # Generate motor command (same as motor_controller would)
        motor_cmd = self._compute_motor_command(x, y)
        if motor_cmd is None:
            print(f"Already at ({x:.3f}, {y:.3f})")
            return
        
        # Print the motor command that would be sent
        print(f"Motor command: {motor_cmd.to_serial()}")
        
        # Find max rotations for duration calculation
        max_rotations = 0
        for motor_idx in ACTIVE_MOTORS:
            rotations = motor_cmd.motors[motor_idx][1]
            if rotations > max_rotations:
                max_rotations = rotations
        
        # Duration based on max rotation at max speed
        duration = max_rotations * 60 / self.max_speed_rpm
        duration = max(0.1, min(10.0, duration))
        
        # Store state for animation
        self.state.target_position = target
        self.state.start_cable_lengths = self.state.cable_lengths.copy()
        self.state.motor_command = motor_cmd
        self.state.move_start_time = time.time()
        self.state.move_duration = duration
        self.state.is_moving = True
        
        # Compute cable deltas for logging
        target_lengths = self.geometry.position_to_cable_lengths(target)[:NUM_CABLES]
        delta_lengths = target_lengths - self.state.start_cable_lengths
        
        print(f"Moving to ({x:.3f}, {y:.3f}) - duration: {duration:.2f}s")
        print(f"  Cable deltas: TL={delta_lengths[0]:+.4f} TR={delta_lengths[1]:+.4f} BL={delta_lengths[2]:+.4f}")
    
    def _forward_to_real_robot(self, cmd: str) -> str:
        """Forward command to real robot (proxy mode)."""
        try:
            sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
            sock.connect(REAL_ROBOT_SOCKET_PATH)
            sock.sendall(cmd.encode("utf-8") + b"\n")
            response = sock.recv(1024).decode("utf-8").strip()
            sock.close()
            return response
        except Exception as e:
            return f"ERROR forwarding: {e}"
    
    def _start_command_server(self):
        """Start Unix socket server to receive commands."""
        # Remove existing socket
        if os.path.exists(VISUALIZER_SOCKET_PATH):
            os.remove(VISUALIZER_SOCKET_PATH)
        
        server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        server.bind(VISUALIZER_SOCKET_PATH)
        server.listen(1)
        server.settimeout(1.0)
        
        print(f"Visualizer listening on {VISUALIZER_SOCKET_PATH}")
        print("Use: python command_client.py to send commands")
        print(f"  (Set COMMAND_SOCKET_PATH to {VISUALIZER_SOCKET_PATH})")
        
        while not self.stop_event.is_set():
            try:
                conn, _ = server.accept()
                data = conn.recv(1024).decode("utf-8").strip()
                if data:
                    print(f"Received: {data}")
                    
                    # Queue command for visualization
                    self.command_queue.put(data)
                    
                    # Forward to real robot if proxy mode
                    if self.proxy_mode:
                        response = self._forward_to_real_robot(data)
                        conn.sendall((response + "\n").encode("utf-8"))
                    else:
                        conn.sendall(b"OK (simulated)\n")
                conn.close()
            except socket.timeout:
                continue
            except Exception as e:
                if not self.stop_event.is_set():
                    print(f"Server error: {e}")
        
        server.close()
        if os.path.exists(VISUALIZER_SOCKET_PATH):
            os.remove(VISUALIZER_SOCKET_PATH)
    
    def run(self):
        """Start the visualizer."""
        # Start command server thread
        server_thread = threading.Thread(target=self._start_command_server, daemon=True)
        server_thread.start()
        
        # Setup animation
        self._update_visuals()
        self.anim = FuncAnimation(
            self.fig, self._animate,
            interval=33,  # ~30 FPS
            blit=False,
            cache_frame_data=False
        )
        
        # Show plot (blocking)
        try:
            plt.show()
        except KeyboardInterrupt:
            pass
        finally:
            self.stop_event.set()
            print("\nVisualizer stopped.")


def main():
    parser = argparse.ArgumentParser(description='Window Cleaning Robot Visualizer')
    parser.add_argument('--proxy', action='store_true',
                       help='Proxy mode: forward commands to real robot')
    parser.add_argument('--config', default=CONFIG_PATH,
                       help='Path to config.yaml')
    args = parser.parse_args()
    
    print("=" * 50)
    print("Window Cleaning Robot Visualizer")
    print("=" * 50)
    
    if args.proxy:
        print("Mode: PROXY (forwarding to real robot)")
    else:
        print("Mode: STANDALONE (simulation only)")
    
    print()
    print("To send commands, use a separate terminal:")
    print(f"  export COMMAND_SOCKET_PATH={VISUALIZER_SOCKET_PATH}")
    print("  python command_client.py")
    print()
    print("Or directly:")
    print(f"  echo 'goto 0.5 0.5' | nc -U {VISUALIZER_SOCKET_PATH}")
    print()
    
    visualizer = RobotVisualizer(config_path=args.config, proxy_mode=args.proxy)
    visualizer.run()


if __name__ == "__main__":
    main()
