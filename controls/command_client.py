#!/usr/bin/env python3
"""
Command client for sending commands to the window cleaning robot main loop.

Usage:
    python command_client.py <command>         # Send a single command
    python command_client.py                   # Interactive mode
"""
import socket
import sys

COMMAND_SOCKET_PATH = "/tmp/window_robot_cmd.sock"


def send_command(command: str) -> str:
    """Send a command to the main loop and return response."""
    try:
        sock = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM)
        sock.connect(COMMAND_SOCKET_PATH)
        sock.sendall(command.encode("utf-8") + b"\n")
        response = sock.recv(1024).decode("utf-8").strip()
        sock.close()
        return response
    except FileNotFoundError:
        return "ERROR: Main loop not running (socket not found)"
    except ConnectionRefusedError:
        return "ERROR: Connection refused - is main loop running?"
    except Exception as e:
        return f"ERROR: {e}"


def interactive_mode():
    """Run in interactive mode, accepting commands from stdin."""
    print("Window Robot Command Client")
    print("Type commands to send to the main loop. Ctrl+C to exit.")
    print("-" * 40)
    
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
        # Single command mode
        command = " ".join(sys.argv[1:])
        response = send_command(command)
        print(response)
    else:
        # Interactive mode
        interactive_mode()


if __name__ == "__main__":
    main()
