#!/usr/bin/env python3
"""
Command client for the robot visualizer.

Usage:
    python viz_client.py <command>         # Send a single command
    python viz_client.py                   # Interactive mode
    
Commands:
    goto <x> <y>              Move to absolute position (meters)
    move <up|down|left|right> [distance]  Move in direction
    home                      Return to center
    clean                     Run CLEANING/CHECKING routine
    stop                      Emergency stop
    setpos <x> <y>            Set current position (calibration)
    status                    Show current status
"""
import socket
import sys

VISUALIZER_SOCKET_PATH = "/tmp/window_robot_viz.sock"


def send_command(command: str) -> str:
    """Send a command to the visualizer and return response."""
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(VISUALIZER_SOCKET_PATH)
        sock.sendall(command.encode("utf-8") + b"\n")
        response = sock.recv(1024).decode("utf-8").strip()
        sock.close()
        return response
    except FileNotFoundError:
        return "ERROR: Visualizer not running (socket not found)"
    except ConnectionRefusedError:
        return "ERROR: Connection refused - is visualizer running?"
    except Exception as e:
        return f"ERROR: {e}"


def interactive_mode():
    """Run in interactive mode."""
    print("Robot Visualizer Command Client")
    print("Commands: goto <x> <y>, move <dir> [dist], home, clean, stop, setpos <x> <y>")
    print("Ctrl+C to exit")
    print("-" * 50)
    
    try:
        while True:
            try:
                cmd = input("> ").strip()
                if not cmd:
                    continue
                if cmd.lower() in ("quit", "exit"):
                    break
                response = send_command(cmd)
                print(response)
            except EOFError:
                break
    except KeyboardInterrupt:
        print("\nExiting.")


def main():
    if len(sys.argv) > 1:
        command = " ".join(sys.argv[1:])
        response = send_command(command)
        print(response)
    else:
        interactive_mode()


if __name__ == "__main__":
    main()
