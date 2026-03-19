#!/usr/bin/env python3
"""
Continuously execute square movement commands via the main loop command socket.

Sends commands in this order forever:
    move left 0.2
    move up 0.2
    move right 0.2
    move down 0.2

On Ctrl+C, sends:
    stop
"""

import time

from command_client import send_command


SQUARE_COMMANDS = [
    "move left 0.2",
    "move up 0.2",
    "move right 0.2",
    "move down 0.2",
]


def main() -> None:
    print("Square movement client running. Press Ctrl+C to stop.")

    try:
        while True:
            for command in SQUARE_COMMANDS:
                response = send_command(command)
                print(f"{command} -> {response}")
                time.sleep(7)  # Wait 5 seconds between commands
    except KeyboardInterrupt:
        print("\nCtrl+C detected. Sending stop command...")
        response = send_command("stop")
        print(f"stop -> {response}")
        print("Exited.")


if __name__ == "__main__":
    main()
