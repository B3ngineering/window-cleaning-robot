#!/usr/bin/env python3
"""
Cable-Driven Window Cleaning Robot Simulator - IMPROVED
========================================================

Fixed issues:
- Better FK convergence using trilateration + optimization
- Real-time animated visualization
- More robust numerics

Motor Layout:
  M1 (0, 2) -------- M2 (2, 2)
      |                  |
      |       EE         |
      |                  |
  M4 (0, 0) -------- M3 (2, 0)
"""

import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.patches import Circle
from dataclasses import dataclass
from typing import Tuple, List
import time


@dataclass
class RobotConfig:
    """Robot geometric and physical parameters"""
    # Motor positions (x, z) in meters
    motor1_pos: np.ndarray = np.array([0.0, 2.0])  # Top-left
    motor2_pos: np.ndarray = np.array([2.0, 2.0])  # Top-right
    motor3_pos: np.ndarray = np.array([2.0, 0.0])  # Bottom-right
    motor4_pos: np.ndarray = np.array([0.0, 0.0])  # Bottom-left
    
    # Physical parameters
    ee_mass: float = 1.0  # End effector mass (kg)
    gravity: float = 9.81  # Gravity (m/s^2)
    
    # Cable constraints
    min_cable_length: float = 0.1  # Minimum cable length (m)
    max_cable_length: float = 3.0  # Maximum cable length (m)
    
    # Control parameters
    max_motor_speed: float = 0.5  # Maximum cable retraction/extension speed (m/s)
    
    def get_motor_positions(self) -> np.ndarray:
        """Return all motor positions as 2x4 array"""
        return np.column_stack([
            self.motor1_pos,
            self.motor2_pos,
            self.motor3_pos,
            self.motor4_pos
        ])


class CableRobotKinematics:
    """Handles kinematic calculations for the cable robot"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.motor_positions = config.get_motor_positions()
    
    def inverse_kinematics(self, ee_pos: np.ndarray) -> np.ndarray:
        """
        Calculate cable lengths for a given end effector position
        
        Args:
            ee_pos: End effector position [x, z]
        
        Returns:
            cable_lengths: Array of 4 cable lengths [L1, L2, L3, L4]
        """
        cable_lengths = np.zeros(4)
        for i in range(4):
            delta = ee_pos - self.motor_positions[:, i]
            cable_lengths[i] = np.linalg.norm(delta)
        
        return cable_lengths
    
    def forward_kinematics_trilateration(self, cable_lengths: np.ndarray,
                                         initial_guess: np.ndarray = None) -> np.ndarray:
        """
        Improved FK using weighted trilateration with damped least squares
        
        This method is more robust and converges faster than pure Newton-Raphson
        """
        if initial_guess is None:
            initial_guess = np.array([1.0, 1.0])
        
        pos = initial_guess.copy()
        
        # Damped least squares parameters
        lambda_damping = 0.01
        max_iter = 50
        tolerance = 1e-6
        
        for iteration in range(max_iter):
            # Calculate current cable lengths
            current_lengths = self.inverse_kinematics(pos)
            
            # Residual (error in cable lengths)
            residual = cable_lengths - current_lengths
            
            # Check convergence
            if np.linalg.norm(residual) < tolerance:
                return pos
            
            # Calculate Jacobian
            J = self._calculate_jacobian(pos)
            
            # Damped least squares (Levenberg-Marquardt style)
            # This is more stable than pure least squares
            JtJ = J.T @ J
            Jtr = J.T @ residual
            
            # Add damping to diagonal
            damped_matrix = JtJ + lambda_damping * np.eye(2)
            
            try:
                # Solve for position update
                delta_pos = np.linalg.solve(damped_matrix, Jtr)
                
                # Apply update with step limiting for stability
                step_size = min(1.0, 0.1 / (np.linalg.norm(delta_pos) + 1e-10))
                pos += step_size * delta_pos
                
                # Clamp to workspace bounds to prevent divergence
                pos[0] = np.clip(pos[0], 0.05, 1.95)
                pos[1] = np.clip(pos[1], 0.05, 1.95)
                
            except np.linalg.LinAlgError:
                # If singular, try direct pseudoinverse
                try:
                    delta_pos = np.linalg.pinv(J) @ residual
                    pos += 0.1 * delta_pos  # Small step
                    pos[0] = np.clip(pos[0], 0.05, 1.95)
                    pos[1] = np.clip(pos[1], 0.05, 1.95)
                except:
                    break
        
        # If not converged, return best guess
        return pos
    
    def _calculate_jacobian(self, ee_pos: np.ndarray) -> np.ndarray:
        """
        Calculate Jacobian matrix: dL/dx where L is cable length, x is position
        
        J[i,j] = ∂L_i/∂x_j
        
        For cable i: L_i = ||ee_pos - motor_i||
        ∂L_i/∂x = (x - x_motor_i) / L_i
        ∂L_i/∂z = (z - z_motor_i) / L_i
        """
        J = np.zeros((4, 2))
        
        for i in range(4):
            delta = ee_pos - self.motor_positions[:, i]
            length = np.linalg.norm(delta)
            
            if length > 1e-6:  # Avoid division by zero
                J[i, :] = delta / length
        
        return J
    
    def check_workspace_limits(self, ee_pos: np.ndarray) -> bool:
        """Check if position is within valid workspace"""
        x, z = ee_pos
        
        # Basic bounding box
        if x < 0.1 or x > 1.9 or z < 0.1 or z > 1.9:
            return False
        
        # Check cable length limits
        cable_lengths = self.inverse_kinematics(ee_pos)
        if np.any(cable_lengths < self.config.min_cable_length) or \
           np.any(cable_lengths > self.config.max_cable_length):
            return False
        
        return True


class PIDController:
    """Simple PID controller for each motor"""
    
    def __init__(self, kp: float = 2.0, ki: float = 0.1, kd: float = 0.5):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
        self.integral = 0.0
        self.prev_error = 0.0
    
    def update(self, error: float, dt: float) -> float:
        """
        Calculate control output
        
        Args:
            error: Current error (desired - actual)
            dt: Time step
        
        Returns:
            control: Control signal (velocity command)
        """
        # Anti-windup: limit integral
        self.integral += error * dt
        self.integral = np.clip(self.integral, -1.0, 1.0)
        
        derivative = (error - self.prev_error) / dt if dt > 0 else 0.0
        
        control = self.kp * error + self.ki * self.integral + self.kd * derivative
        
        self.prev_error = error
        
        return control
    
    def reset(self):
        """Reset controller state"""
        self.integral = 0.0
        self.prev_error = 0.0


class CableRobotSimulator:
    """Main simulator class that integrates kinematics and control"""
    
    def __init__(self, config: RobotConfig):
        self.config = config
        self.kinematics = CableRobotKinematics(config)
        
        # Controllers for each cable
        self.controllers = [PIDController() for _ in range(4)]
        
        # State variables
        self.ee_position = np.array([1.0, 1.0])  # Start at center
        self.ee_velocity = np.array([0.0, 0.0])
        self.cable_lengths = self.kinematics.inverse_kinematics(self.ee_position)
        self.cable_velocities = np.zeros(4)
        
        # Desired state
        self.desired_position = self.ee_position.copy()
        self.desired_cable_lengths = self.cable_lengths.copy()
        
        # Simulation time
        self.time = 0.0
        self.dt = 0.01  # 100 Hz simulation
        
        # History for plotting
        self.history = {
            'time': [],
            'ee_pos': [],
            'desired_pos': [],
            'cable_lengths': [],
            'desired_cable_lengths': [],
            'position_error': []
        }
    
    def set_desired_position(self, position: np.ndarray):
        """Set desired end effector position"""
        if self.kinematics.check_workspace_limits(position):
            self.desired_position = position.copy()
            self.desired_cable_lengths = self.kinematics.inverse_kinematics(position)
            return True
        else:
            print(f"Warning: Position {position} is outside workspace limits")
            return False
    
    def step(self):
        """Execute one simulation step"""
        
        # 1. Calculate cable length errors
        errors = self.desired_cable_lengths - self.cable_lengths
        
        # 2. Calculate motor velocities using PID control
        motor_velocities = np.zeros(4)
        for i in range(4):
            motor_velocities[i] = self.controllers[i].update(errors[i], self.dt)
            
            # Saturate to max motor speed
            motor_velocities[i] = np.clip(motor_velocities[i],
                                         -self.config.max_motor_speed,
                                         self.config.max_motor_speed)
        
        # 3. Update cable lengths
        self.cable_lengths += motor_velocities * self.dt
        
        # Ensure cable lengths stay within physical limits
        self.cable_lengths = np.clip(self.cable_lengths,
                                     self.config.min_cable_length,
                                     self.config.max_cable_length)
        
        # 4. Calculate end effector position from cable lengths (forward kinematics)
        self.ee_position = self.kinematics.forward_kinematics_trilateration(
            self.cable_lengths,
            initial_guess=self.ee_position
        )
        
        # 5. Update time
        self.time += self.dt
        
        # 6. Record history
        position_error = np.linalg.norm(self.ee_position - self.desired_position)
        self.history['time'].append(self.time)
        self.history['ee_pos'].append(self.ee_position.copy())
        self.history['desired_pos'].append(self.desired_position.copy())
        self.history['cable_lengths'].append(self.cable_lengths.copy())
        self.history['desired_cable_lengths'].append(self.desired_cable_lengths.copy())
        self.history['position_error'].append(position_error)
    
    def run_to_target(self, target: np.ndarray, timeout: float = 10.0, 
                     position_tolerance: float = 0.01):
        """Run simulation until target is reached or timeout"""
        if not self.set_desired_position(target):
            return False
        
        start_time = self.time
        while self.time - start_time < timeout:
            self.step()
            
            # Check if reached target
            error = np.linalg.norm(self.ee_position - self.desired_position)
            if error < position_tolerance:
                print(f"✓ Reached target in {self.time - start_time:.2f} seconds (error: {error:.4f} m)")
                return True
        
        error = np.linalg.norm(self.ee_position - self.desired_position)
        print(f"✗ Timeout: Final error = {error:.4f} m")
        return False
    
    def get_state_string(self) -> str:
        """Return current state as formatted string"""
        return f"""Time: {self.time:.2f} s

EE Position: [{self.ee_position[0]:.3f}, {self.ee_position[1]:.3f}]
Desired: [{self.desired_position[0]:.3f}, {self.desired_position[1]:.3f}]
Error: {np.linalg.norm(self.ee_position - self.desired_position):.4f} m
Cables: [{self.cable_lengths[0]:.3f}, {self.cable_lengths[1]:.3f}, {self.cable_lengths[2]:.3f}, {self.cable_lengths[3]:.3f}]"""


class RealTimeVisualizer:
    """Real-time animated visualization of the cable robot"""
    
    def __init__(self, sim: CableRobotSimulator, enable_keyboard: bool = True):
        self.sim = sim
        self.enable_keyboard = enable_keyboard
        self.move_step = 0.05  # Movement step size in meters
        
        # Create figure with 3 subplots
        self.fig, (self.ax_robot, self.ax_error, self.ax_cables) = plt.subplots(
            1, 3, figsize=(18, 6)
        )
        
        # Setup robot plot
        self._setup_robot_plot()
        
        # Setup error plot
        self._setup_error_plot()
        
        # Setup cable lengths plot
        self._setup_cable_plot()
        
        # Connect keyboard events
        if self.enable_keyboard:
            self.fig.canvas.mpl_connect('key_press_event', self._on_key_press)
        
        self.animation = None
    
    def _on_key_press(self, event):
        """Handle keyboard input for manual control"""
        current_target = self.sim.desired_position.copy()
        
        if event.key == 'up':
            current_target[1] += self.move_step  # Move up (increase z)
        elif event.key == 'down':
            current_target[1] -= self.move_step  # Move down (decrease z)
        elif event.key == 'left':
            current_target[0] -= self.move_step  # Move left (decrease x)
        elif event.key == 'right':
            current_target[0] += self.move_step  # Move right (increase x)
        elif event.key == 'c':
            # Return to center
            current_target = np.array([1.0, 1.0])
            print("→ Returning to center [1.0, 1.0]")
        elif event.key == 'r':
            # Reset controllers
            for controller in self.sim.controllers:
                controller.reset()
            print("→ Controllers reset")
            return
        else:
            return  # Ignore other keys
        
        # Try to set the new position
        if self.sim.set_desired_position(current_target):
            print(f"→ Target: [{current_target[0]:.2f}, {current_target[1]:.2f}]")
    
    def _setup_robot_plot(self):
        """Setup the robot visualization subplot"""
        ax = self.ax_robot
        
        # Window frame
        frame_x = [0, 2, 2, 0, 0]
        frame_z = [0, 0, 2, 2, 0]
        ax.plot(frame_x, frame_z, 'b-', linewidth=2, label='Window')
        
        # Motors
        motors = self.sim.config.get_motor_positions()
        ax.plot(motors[0, :], motors[1, :], 'ro', markersize=12, label='Motors', zorder=5)
        
        # Motor labels
        labels = ['M1', 'M2', 'M3', 'M4']
        for i, label in enumerate(labels):
            ax.text(motors[0, i], motors[1, i] + 0.12, label,
                   ha='center', fontsize=10, fontweight='bold')
        
        # Cable lines (will be updated)
        self.cable_lines = []
        for i in range(4):
            line, = ax.plot([], [], 'k-', linewidth=1.5, alpha=0.6)
            self.cable_lines.append(line)
        
        # End effector
        self.ee_point, = ax.plot([], [], 'gs', markersize=20, label='End Effector', zorder=10)
        self.ee_trail, = ax.plot([], [], 'g-', linewidth=1, alpha=0.3)
        
        # Target marker
        self.target_point, = ax.plot([], [], 'r*', markersize=25, label='Target', zorder=9)
        
        # Text for info
        self.info_text = ax.text(0.02, 0.98, '', transform=ax.transAxes,
                                verticalalignment='top', fontsize=9,
                                bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        
        ax.set_xlabel('X Position (m)', fontsize=11)
        ax.set_ylabel('Z Position (m)', fontsize=11)
        ax.set_title('Cable Robot Real-Time Simulation', fontsize=13, fontweight='bold')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_aspect('equal')
        ax.set_xlim(-0.3, 2.3)
        ax.set_ylim(-0.3, 2.3)
        
        # Trail history
        self.trail_x = []
        self.trail_y = []
    
    def _setup_error_plot(self):
        """Setup the error plot subplot"""
        ax = self.ax_error
        
        self.error_line, = ax.plot([], [], 'r-', linewidth=2)
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Position Error (m)', fontsize=11)
        ax.set_title('Tracking Error', fontsize=13, fontweight='bold')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 0.5)
    
    def _setup_cable_plot(self):
        """Setup the cable lengths plot subplot"""
        ax = self.ax_cables
        
        colors = ['#e41a1c', '#377eb8', '#4daf4a', '#984ea3']  # Distinct colors
        self.cable_lines_plot = []
        self.cable_desired_lines = []
        
        for i in range(4):
            # Actual cable length (solid line)
            line, = ax.plot([], [], '-', color=colors[i], linewidth=2, 
                           label=f'Cable {i+1}')
            self.cable_lines_plot.append(line)
            
            # Desired cable length (dashed line)
            line_desired, = ax.plot([], [], '--', color=colors[i], linewidth=1, 
                                    alpha=0.5)
            self.cable_desired_lines.append(line_desired)
        
        ax.set_xlabel('Time (s)', fontsize=11)
        ax.set_ylabel('Cable Length (m)', fontsize=11)
        ax.set_title('Cable Lengths (solid=actual, dashed=desired)', fontsize=13, fontweight='bold')
        ax.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
        ax.set_xlim(0, 1)
        ax.set_ylim(0, 3)
    
    def update_frame(self, frame):
        """Update animation frame"""
        # Run simulation step
        self.sim.step()
        
        # Update cable lines
        motors = self.sim.config.get_motor_positions()
        for i, line in enumerate(self.cable_lines):
            line.set_data([motors[0, i], self.sim.ee_position[0]],
                         [motors[1, i], self.sim.ee_position[1]])
        
        # Update end effector
        self.ee_point.set_data([self.sim.ee_position[0]], [self.sim.ee_position[1]])
        
        # Update trail
        self.trail_x.append(self.sim.ee_position[0])
        self.trail_y.append(self.sim.ee_position[1])
        if len(self.trail_x) > 200:  # Keep last 200 points
            self.trail_x.pop(0)
            self.trail_y.pop(0)
        self.ee_trail.set_data(self.trail_x, self.trail_y)
        
        # Update target
        self.target_point.set_data([self.sim.desired_position[0]],
                                   [self.sim.desired_position[1]])
        
        # Update info text
        error = np.linalg.norm(self.sim.ee_position - self.sim.desired_position)
        info_str = f"Time: {self.sim.time:.2f} s\n"
        info_str += f"Position: [{self.sim.ee_position[0]:.3f}, {self.sim.ee_position[1]:.3f}]\n"
        info_str += f"Error: {error:.4f} m"
        self.info_text.set_text(info_str)
        
        # Update error plot
        if len(self.sim.history['time']) > 0:
            times = self.sim.history['time']
            errors = self.sim.history['position_error']
            self.error_line.set_data(times, errors)
            
            # Auto-scale
            if len(times) > 1:
                self.ax_error.set_xlim(0, max(10, times[-1]))
                max_error = max(errors) if errors else 0.1
                self.ax_error.set_ylim(0, max(0.5, max_error * 1.2))
            
            # Update cable lengths plot
            cable_lengths_history = np.array(self.sim.history['cable_lengths'])
            desired_cable_history = np.array(self.sim.history['desired_cable_lengths'])
            
            if len(cable_lengths_history) > 0:
                for i in range(4):
                    self.cable_lines_plot[i].set_data(times, cable_lengths_history[:, i])
                    self.cable_desired_lines[i].set_data(times, desired_cable_history[:, i])
                
                # Auto-scale cable plot
                self.ax_cables.set_xlim(0, max(10, times[-1]))
                all_lengths = np.concatenate([cable_lengths_history.flatten(), 
                                              desired_cable_history.flatten()])
                min_len = max(0, np.min(all_lengths) - 0.1)
                max_len = np.max(all_lengths) + 0.1
                self.ax_cables.set_ylim(min_len, max_len)
        
        return (self.cable_lines + [self.ee_point, self.ee_trail, 
                self.target_point, self.info_text, self.error_line] +
                self.cable_lines_plot + self.cable_desired_lines)
    
    def animate(self, duration: float = 20.0):
        """
        Start real-time animation
        
        Args:
            duration: Animation duration in seconds
        """
        frames = int(duration / self.sim.dt)
        
        self.animation = FuncAnimation(
            self.fig,
            self.update_frame,
            frames=frames,
            interval=self.sim.dt * 1000,  # milliseconds
            blit=True,
            repeat=False
        )
        
        plt.tight_layout()
        plt.show()


def demo_real_time():
    """Demo with real-time visualization"""
    print("="*60)
    print("Real-Time Cable Robot Simulator")
    print("="*60)
    print("\nKeyboard Controls:")
    print("  ↑/↓/←/→  : Move target position")
    print("  C        : Return to center")
    print("  R        : Reset controllers")
    print("="*60)
    
    config = RobotConfig()
    sim = CableRobotSimulator(config)
    
    # Set up a trajectory
    print("\nSetting up trajectory...")
    
    # The visualizer will run the simulation
    viz = RealTimeVisualizer(sim)
    
    # Set initial target
    sim.set_desired_position(np.array([0.6, 1.5]))
    
    # Start animation (this will run for 20 seconds)
    print("Starting animation... Close the window to end.\n")
    
    # After 5 seconds, change target (we'll use a timer in the animation)
    def change_targets():
        waypoints = [
            np.array([0.6, 1.5]),
            np.array([1.5, 1.5]),
            np.array([1.5, 0.6]),
            np.array([0.6, 0.6]),
            np.array([1.0, 1.0]),
        ]
        
        times_to_change = [0, 4, 8, 12, 16]
        
        for i, (wp, t) in enumerate(zip(waypoints, times_to_change)):
            if sim.time >= t:
                sim.set_desired_position(wp)
    
    # Modify update function to include target changes
    original_update = viz.update_frame
    def update_with_targets(frame):
        # Change targets at specific times
        if 0 <= sim.time < 0.1:
            sim.set_desired_position(np.array([0.6, 1.5]))
        elif 4 <= sim.time < 4.1:
            print(f"→ New target: [1.5, 1.5]")
            sim.set_desired_position(np.array([1.5, 1.5]))
        elif 8 <= sim.time < 8.1:
            print(f"→ New target: [1.5, 0.6]")
            sim.set_desired_position(np.array([1.5, 0.6]))
        elif 12 <= sim.time < 12.1:
            print(f"→ New target: [0.6, 0.6]")
            sim.set_desired_position(np.array([0.6, 0.6]))
        elif 16 <= sim.time < 16.1:
            print(f"→ New target: [1.0, 1.0] (center)")
            sim.set_desired_position(np.array([1.0, 1.0]))
        
        return original_update(frame)
    
    viz.update_frame = update_with_targets
    
    viz.animate(duration=20.0)
    
    print("\nAnimation complete!")
    print(f"Final position: {sim.ee_position}")
    print(f"Final error: {np.linalg.norm(sim.ee_position - sim.desired_position):.4f} m")


if __name__ == "__main__":
    demo_real_time()