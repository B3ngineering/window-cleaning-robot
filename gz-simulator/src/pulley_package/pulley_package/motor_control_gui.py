#!/usr/bin/env python3
"""
Motor Control GUI - Arrow-based directional control for cable robot.
Zero gravity / ideal cable model.
"""

import tkinter as tk
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import threading


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_gui')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/motor_velocity_controller/commands',
            10
        )
        # Motor velocities: [top_left, top_right, bottom_left, bottom_right]
        self.velocities = [0.0, 0.0, 0.0, 0.0]
        
        # Publish at 10Hz
        self.timer = self.create_timer(0.1, self.publish_velocities)
        self.get_logger().info('Motor Control GUI started (zero gravity mode)')
    
    def publish_velocities(self):
        msg = Float64MultiArray()
        msg.data = self.velocities
        self.publisher.publish(msg)
    
    def set_velocities(self, velocities: list):
        self.velocities = velocities
        self.get_logger().info(f'Motors [TL, TR, BL, BR]: {[f"{v:.1f}" for v in velocities]}')
    
    def stop(self):
        self.velocities = [0.0, 0.0, 0.0, 0.0]


class MotorControlGUI:
    def __init__(self, node: MotorControlNode):
        self.node = node
        self.speed = 5.0  # rad/s (higher default since no gravity resistance)
        
        self.root = tk.Tk()
        self.root.title("Cable Robot Control (Zero Gravity)")
        self.root.geometry("400x500")
        self.root.configure(bg='#2c3e50')
        self.root.minsize(400, 500)
        
        self._create_widgets()
        self._bind_keys()
    
    def _create_widgets(self):
        # Title
        tk.Label(
            self.root,
            text="Cable Robot Control",
            font=('Helvetica', 18, 'bold'),
            bg='#2c3e50',
            fg='white'
        ).pack(pady=(20, 5))
        
        # Mode indicator
        tk.Label(
            self.root,
            text="⚡ Zero Gravity Mode ⚡",
            font=('Helvetica', 11, 'bold'),
            bg='#2c3e50',
            fg='#f1c40f'
        ).pack(pady=(0, 10))
        
        # Instructions
        tk.Label(
            self.root,
            text="Use arrow keys or click buttons\nPlatform moves freely - no gravity!",
            font=('Helvetica', 10),
            bg='#2c3e50',
            fg='#bdc3c7'
        ).pack(pady=(0, 20))
        
        # Arrow buttons frame
        arrow_frame = tk.Frame(self.root, bg='#2c3e50')
        arrow_frame.pack(pady=20)
        
        btn_style = {
            'font': ('Helvetica', 28),
            'width': 2,
            'height': 1,
            'bg': '#3498db',
            'fg': 'white',
            'activebackground': '#2980b9',
            'activeforeground': 'white',
            'relief': 'raised',
            'bd': 3
        }
        
        # Up button
        self.btn_up = tk.Button(arrow_frame, text="▲", **btn_style)
        self.btn_up.grid(row=0, column=1, padx=5, pady=5)
        self.btn_up.bind('<ButtonPress-1>', lambda e: self._move('up'))
        self.btn_up.bind('<ButtonRelease-1>', lambda e: self._stop())
        
        # Left button
        self.btn_left = tk.Button(arrow_frame, text="◀", **btn_style)
        self.btn_left.grid(row=1, column=0, padx=5, pady=5)
        self.btn_left.bind('<ButtonPress-1>', lambda e: self._move('left'))
        self.btn_left.bind('<ButtonRelease-1>', lambda e: self._stop())
        
        # Stop button (center)
        self.btn_stop = tk.Button(
            arrow_frame, text="⏹", 
            font=('Helvetica', 28),
            width=2, height=1,
            bg='#e74c3c', fg='white',
            activebackground='#c0392b',
            relief='raised', bd=3,
            command=self._stop
        )
        self.btn_stop.grid(row=1, column=1, padx=5, pady=5)
        
        # Right button
        self.btn_right = tk.Button(arrow_frame, text="▶", **btn_style)
        self.btn_right.grid(row=1, column=2, padx=5, pady=5)
        self.btn_right.bind('<ButtonPress-1>', lambda e: self._move('right'))
        self.btn_right.bind('<ButtonRelease-1>', lambda e: self._stop())
        
        # Down button
        self.btn_down = tk.Button(arrow_frame, text="▼", **btn_style)
        self.btn_down.grid(row=2, column=1, padx=5, pady=5)
        self.btn_down.bind('<ButtonPress-1>', lambda e: self._move('down'))
        self.btn_down.bind('<ButtonRelease-1>', lambda e: self._stop())
        
        # Speed control frame
        speed_frame = tk.Frame(self.root, bg='#2c3e50')
        speed_frame.pack(pady=20)
        
        tk.Label(
            speed_frame,
            text="Speed:",
            font=('Helvetica', 12),
            bg='#2c3e50',
            fg='white'
        ).pack(side='left', padx=5)
        
        self.speed_var = tk.DoubleVar(value=self.speed)
        speed_scale = tk.Scale(
            speed_frame,
            from_=1.0, to=10.0,
            orient='horizontal',
            variable=self.speed_var,
            command=self._update_speed,
            length=150,
            bg='#34495e',
            fg='white',
            highlightthickness=0,
            troughcolor='#2c3e50'
        )
        speed_scale.pack(side='left', padx=5)
        
        tk.Label(
            speed_frame,
            text="rad/s",
            font=('Helvetica', 10),
            bg='#2c3e50',
            fg='#bdc3c7'
        ).pack(side='left')
        
        # Status label
        self.status_var = tk.StringVar(value="Ready")
        tk.Label(
            self.root,
            textvariable=self.status_var,
            font=('Helvetica', 12),
            bg='#2c3e50',
            fg='#2ecc71'
        ).pack(pady=10)
    
    def _bind_keys(self):
        """Bind keyboard arrow keys."""
        self.root.bind('<KeyPress-Up>', lambda e: self._move('up'))
        self.root.bind('<KeyRelease-Up>', lambda e: self._stop())
        self.root.bind('<KeyPress-Down>', lambda e: self._move('down'))
        self.root.bind('<KeyRelease-Down>', lambda e: self._stop())
        self.root.bind('<KeyPress-Left>', lambda e: self._move('left'))
        self.root.bind('<KeyRelease-Left>', lambda e: self._stop())
        self.root.bind('<KeyPress-Right>', lambda e: self._move('right'))
        self.root.bind('<KeyRelease-Right>', lambda e: self._stop())
        self.root.bind('<space>', lambda e: self._stop())
    
    def _update_speed(self, value):
        self.speed = float(value)
    
    def _move(self, direction: str):
        """Send motor commands for the given direction."""
        s = self.speed
        
        # Motor order: [top_left, top_right, bottom_left, bottom_right]
        # 
        # Ideal cable model (zero gravity):
        # - Positive motor = pull cable = attract platform toward that pulley
        # - To move in a direction, pull from that side
        #
        # UP:    Pull from top (TL+, TR+), release from bottom (BL-, BR-)
        # DOWN:  Pull from bottom (BL+, BR+), release from top (TL-, TR-)
        # LEFT:  Pull from left (TL+, BL+), release from right (TR-, BR-)
        # RIGHT: Pull from right (TR+, BR+), release from left (TL-, BL-)
        
        commands = {
            'up':    [s,  s,  -s, -s],
            'down':  [-s, -s,  s,  s],
            'left':  [s,  -s,  s, -s],
            'right': [-s,  s, -s,  s],
        }
        
        velocities = commands.get(direction, [0, 0, 0, 0])
        self.node.set_velocities(velocities)
        self.status_var.set(f"→ {direction.upper()}")
    
    def _stop(self):
        """Stop all motors - platform will stay in place (zero gravity)."""
        self.node.stop()
        self.status_var.set("● Holding Position")
    
    def run(self):
        self.root.mainloop()


def main():
    rclpy.init()
    node = MotorControlNode()
    
    # Run ROS2 in background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()
    
    # Run GUI
    gui = MotorControlGUI(node)
    
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
