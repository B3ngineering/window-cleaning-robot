import os
import queue
import socket
import threading
import time

from camera import OnboardCameraStream
from serial_stream import SerialStream
from motor_controller import MotorController

COMMAND_SOCKET_PATH = "/tmp/window_robot_cmd.sock"
CONFIG_PATH = os.path.join(os.path.dirname(__file__), "config.yaml")


def start_command_server(event_queue, stop_event):
    """Start a Unix socket server to receive commands."""
    import os
    
    # Remove existing socket file if present
    if os.path.exists(COMMAND_SOCKET_PATH):
        os.remove(COMMAND_SOCKET_PATH)
    
    server = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
    server.bind(COMMAND_SOCKET_PATH)
    server.listen(1)
    server.settimeout(1.0)
    
    while not stop_event.is_set():
        try:
            conn, _ = server.accept()
            data = conn.recv(1024).decode("utf-8").strip()
            if data:
                event_queue.put({
                    "source": "command",
                    "data": {"value": data, "timestamp": time.time()}
                })
                conn.sendall(b"OK\n")
            conn.close()
        except socket.timeout:
            continue
        except Exception as e:
            if not stop_event.is_set():
                print(f"Command server error: {e}")
    
    server.close()
    if os.path.exists(COMMAND_SOCKET_PATH):
        os.remove(COMMAND_SOCKET_PATH)


def main():
    event_queue = queue.Queue()
    stop_event = threading.Event()

    # Initial mode is IDLE
    mode = 'IDLE'
    
    # Start command server thread for controlling robot via CLI
    cmd_thread = threading.Thread(
        target=start_command_server,
        args=(event_queue, stop_event),
        daemon=True
    )
    cmd_thread.start()
    print(f"Command server listening on {COMMAND_SOCKET_PATH}")
    
    # Initialize motor controller with config and connect to motor STM32
    motor_controller = MotorController(config_path=CONFIG_PATH)
    print(f"Motor controller initialized at position {motor_controller.position}")
    
    # Connect to motor STM32 (separate board from sensors)
    if motor_controller.connect():
        print("Motor STM32 connected")
    else:
        print("Warning: Motor STM32 not connected - commands will be logged only")
    
    # Camera and sensor serial streams (optional - may not be available)
    camera = None
    serial_stream = None
    
    # Connect to onboard camera for dirt detection
    try:
        camera = OnboardCameraStream(
            dirty_threshold=0.7,
            event_queue=event_queue,
        )
    except Exception as e:
        print(f"Camera not available: {e}")
    
    # Connect to sensor STM32 (separate from motor controller)
    try:
        serial_stream = SerialStream(
            port="/dev/ttyUSB1",  # Sensor port from config
            baudrate=115200,
            event_queue=event_queue,
        ).start()
        print("Sensor STM32 connected on /dev/ttyUSB1")
    except Exception as e:
        print(f"Sensor serial not available: {e}")

    # Main event loop
    try:
        while True:

            # Logic for mode handling
            # If mode = CHECKING, turn on camera if not already
            # If mode = CLEANING, move roller motor if not already in position
            # If mode = STOP, await commands until mode changes
            # If mode = MOVING, await arrival at target or obstacle detection

            # Event queue source - reading from camera and serial stream
            # Eventually we'll take motor events as well
            event = event_queue.get()
            source = event["source"]
            data = event["data"]

            clean_frames = 0
            dirty_frames = 0

            # Handle commands from the client
            if source == "command":
                cmd_str = data["value"]
                cmd = cmd_str.lower().split()
                if not cmd:
                    continue
                
                action = cmd[0]
                args = cmd[1:]
                
                if action == "goto":
                    # goto <x> <y> - move to absolute position
                    if len(args) >= 2:
                        try:
                            x, y = float(args[0]), float(args[1])
                            valid, msg = motor_controller.validate_target(x, y)
                            if valid:
                                print(f"Moving to ({x:.3f}, {y:.3f})")
                                motor_controller.goto(x, y)
                                print(f"Arrived at ({x:.3f}, {y:.3f})")
                            else:
                                print(f"Invalid target: {msg}")
                        except ValueError:
                            print("Usage: goto <x> <y> (coordinates in meters)")
                    else:
                        print("Usage: goto <x> <y>")
                
                elif action == "move":
                    # move <direction> [distance] - move in cardinal direction
                    if len(args) >= 1:
                        direction = args[0]
                        distance = float(args[1]) if len(args) > 1 else 0.05
                        print(f"Moving {direction} by {distance:.3f}m")
                        if motor_controller.move_direction(direction, distance):
                            pos = motor_controller.position
                            print(f"Now at ({pos[0]:.3f}, {pos[1]:.3f})")
                    else:
                        print("Usage: move <up|down|left|right> [distance]")
                
                elif action == "home":
                    print("Returning to home position")
                    motor_controller.home()
                    pos = motor_controller.position
                    print(f"Home: ({pos[0]:.3f}, {pos[1]:.3f})")
                
                elif action == "stop":
                    print("Emergency stop")
                    motor_controller.stop()
                    mode = 'IDLE'
                
                elif action == "pause":
                    print("Pausing")
                    motor_controller.pause()
                    mode = 'PAUSED'
                
                elif action == "resume":
                    print("Resuming")
                    motor_controller.resume()
                    mode = 'IDLE'
                
                elif action == "status":
                    status = motor_controller.get_status()
                    pos = status['position']
                    print(f"Position: ({pos['x']:.3f}, {pos['y']:.3f})")
                    print(f"Cables: {[f'{l:.3f}' for l in status['cable_lengths']]}")
                    print(f"Mode: {mode}, Moving: {status['is_moving']}, Paused: {status['paused']}")
                
                elif action == "setpos":
                    # setpos <x> <y> - manually set current position (calibration)
                    if len(args) >= 2:
                        try:
                            x, y = float(args[0]), float(args[1])
                            if motor_controller.set_position(x, y):
                                print(f"Position set to ({x:.3f}, {y:.3f})")
                            else:
                                print("Failed to set position")
                        except ValueError:
                            print("Usage: setpos <x> <y>")
                    else:
                        print("Usage: setpos <x> <y>")
                
                elif action == "help":
                    print("Commands:")
                    print("  goto <x> <y>        - Move to position (meters)")
                    print("  move <dir> [dist]   - Move direction (up/down/left/right)")
                    print("  home                - Return to center")
                    print("  stop                - Emergency stop")
                    print("  pause / resume      - Pause/resume operation")
                    print("  status              - Show position and state")
                    print("  setpos <x> <y>      - Set current position (calibration)")
                
                else:
                    print(f"Unknown command: {action}. Type 'help' for commands.")

            # Sensor interrupt logic
            elif source == "serial":
                values = data["value"]
                lowest = min(values)
                print(f"serial={values}")
                # If we see an obstacle, stop the motors and await commands
                if lowest < 300:  # Threshold for obstacle detection (in mm)
                    print("Obstacle detected! Stopping motors.")
                    motor_controller.stop()
                    mode = 'STOP'
            
            elif source == "camera":
                label = data["value"]
                if label == 0:
                    dirty_frames += 1
                    clean_frames = 0
                else:
                    clean_frames += 1
                    dirty_frames = 0
                
                if dirty_frames >= 3:
                    print("Dirt detected! Recleaning.")
                    motor_controller.stop()
                    mode = 'DIRTY'

            
    except KeyboardInterrupt:
        pass
    finally:
        stop_event.set()
        motor_controller.stop()
        motor_controller.disconnect()
        if camera:
            camera.stop()
        if serial_stream:
            serial_stream.stop()


if __name__ == "__main__":
    main()
