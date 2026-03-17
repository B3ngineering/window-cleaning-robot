import threading
import time
import tkinter as tk
from tkinter import ttk

import serial


STEPS_PER_SEC = 300
STEPS_PER_REV = 3200

MOTOR_MAP = {
    "BL": {"id": 3, "directions": {"wind": 0, "unwind": 1}},
    "TL": {"id": 2, "directions": {"wind": 0, "unwind": 1}},
    "BR": {"id": 1, "directions": {"unwind": 0, "wind": 1}},
    "TR": {"id": 4, "directions": {"unwind": 0, "wind": 1}},
}


def build_packet(motor_id, direction, steps_per_sec, pulses):
    speed_enc = int(steps_per_sec) // 10
    return bytes([
        1,
        int(motor_id),
        int(direction),
        (speed_enc >> 8) & 0xFF,
        speed_enc & 0xFF,
        (int(pulses) >> 8) & 0xFF,
        int(pulses) & 0xFF,
    ])


class CalibrationGui:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Static Calibration")
        self.ser = None
        self.reader_stop = threading.Event()
        self.reader_thread = None

        self.port_var = tk.StringVar(value="/dev/ttyACM0")
        self.baud_var = tk.StringVar(value="115200")
        self.motor_var = tk.StringVar(value="BL")
        self.mode_var = tk.StringVar(value="wind")
        self.rotation_var = tk.DoubleVar(value=0.25)

        self._build_ui()
        self._refresh_direction_label()

    def _build_ui(self):
        frame = ttk.Frame(self.root, padding=10)
        frame.grid(row=0, column=0, sticky="nsew")

        ttk.Label(frame, text="Port").grid(row=0, column=0, sticky="w")
        ttk.Entry(frame, textvariable=self.port_var, width=18).grid(row=0, column=1, padx=6, sticky="w")

        ttk.Label(frame, text="Baud").grid(row=0, column=2, sticky="w")
        ttk.Entry(frame, textvariable=self.baud_var, width=10).grid(row=0, column=3, padx=6, sticky="w")

        self.connect_btn = ttk.Button(frame, text="Connect", command=self.toggle_connection)
        self.connect_btn.grid(row=0, column=4, padx=6)

        ttk.Label(frame, text="Motor").grid(row=1, column=0, sticky="w", pady=(10, 0))
        motor_menu = ttk.Combobox(frame, textvariable=self.motor_var, values=list(MOTOR_MAP.keys()), state="readonly", width=8)
        motor_menu.grid(row=1, column=1, sticky="w", pady=(10, 0))
        motor_menu.bind("<<ComboboxSelected>>", lambda _e: self._refresh_direction_label())

        ttk.Label(frame, text="Mode").grid(row=1, column=2, sticky="w", pady=(10, 0))
        mode_menu = ttk.Combobox(frame, textvariable=self.mode_var, values=["wind", "unwind"], state="readonly", width=10)
        mode_menu.grid(row=1, column=3, sticky="w", pady=(10, 0))
        mode_menu.bind("<<ComboboxSelected>>", lambda _e: self._refresh_direction_label())

        self.direction_info = ttk.Label(frame, text="")
        self.direction_info.grid(row=2, column=0, columnspan=5, sticky="w", pady=(6, 0))

        ttk.Label(frame, text="Rotations (0.25 to 2.00)").grid(row=3, column=0, columnspan=2, sticky="w", pady=(10, 0))
        rotation_spin = ttk.Spinbox(
            frame,
            from_=0.25,
            to=2.0,
            increment=0.25,
            textvariable=self.rotation_var,
            width=8,
            format="%.2f",
        )
        rotation_spin.grid(row=3, column=2, sticky="w", pady=(10, 0))

        self.send_btn = ttk.Button(frame, text="Send Static Command", command=self.send_command, state="disabled")
        self.send_btn.grid(row=3, column=3, columnspan=2, sticky="ew", pady=(10, 0))

        ttk.Label(frame, text=f"Speed is fixed at {STEPS_PER_SEC} steps/sec").grid(
            row=4,
            column=0,
            columnspan=5,
            sticky="w",
            pady=(8, 0),
        )

        self.log = tk.Text(frame, height=14, width=72, state="disabled")
        self.log.grid(row=5, column=0, columnspan=5, sticky="nsew", pady=(10, 0))

        self.root.protocol("WM_DELETE_WINDOW", self.on_close)
        self.root.rowconfigure(0, weight=1)
        self.root.columnconfigure(0, weight=1)
        frame.rowconfigure(5, weight=1)
        frame.columnconfigure(4, weight=1)

    def _refresh_direction_label(self):
        motor_key = self.motor_var.get()
        mode = self.mode_var.get()
        motor = MOTOR_MAP[motor_key]
        direction = motor["directions"][mode]
        self.direction_info.config(
            text=f"Legend: {motor_key} -> motor_id={motor['id']}, {mode}=direction {direction}"
        )

    def append_log(self, text):
        self.log.configure(state="normal")
        self.log.insert("end", text + "\n")
        self.log.see("end")
        self.log.configure(state="disabled")

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self._disconnect()
            return

        try:
            self.ser = serial.Serial(self.port_var.get().strip(), int(self.baud_var.get().strip()), timeout=0.2)
            time.sleep(0.1)
            self.connect_btn.config(text="Disconnect")
            self.send_btn.config(state="normal")
            self.reader_stop.clear()
            self.reader_thread = threading.Thread(target=self._read_serial_loop, daemon=True)
            self.reader_thread.start()
            self.append_log(f"Connected to {self.ser.port} @ {self.ser.baudrate}")
        except Exception as exc:
            self.append_log(f"Connect failed: {exc}")
            self.ser = None

    def _disconnect(self):
        self.reader_stop.set()
        if self.ser:
            try:
                if self.ser.is_open:
                    self.ser.close()
            except Exception:
                pass
        self.ser = None
        self.connect_btn.config(text="Connect")
        self.send_btn.config(state="disabled")
        self.append_log("Disconnected")

    def _read_serial_loop(self):
        while not self.reader_stop.is_set() and self.ser and self.ser.is_open:
            try:
                line = self.ser.readline()
                if line:
                    text = line.decode("utf-8", errors="replace").strip()
                    if text:
                        self.root.after(0, self.append_log, f"RX: {text}")
            except Exception as exc:
                self.root.after(0, self.append_log, f"Serial read error: {exc}")
                break

    def send_command(self):
        if not (self.ser and self.ser.is_open):
            self.append_log("Not connected")
            return

        motor_key = self.motor_var.get()
        mode = self.mode_var.get()
        rotations = float(self.rotation_var.get())
        if rotations < 0.25 or rotations > 2.0:
            self.append_log("Rotations must be between 0.25 and 2.0")
            return

        motor = MOTOR_MAP[motor_key]
        motor_id = motor["id"]
        direction = motor["directions"][mode]
        pulses = int(round(rotations * STEPS_PER_REV))

        packet = build_packet(motor_id, direction, STEPS_PER_SEC, pulses)

        try:
            self.ser.write(packet)
            self.append_log(
                f"TX: motor={motor_key}(id={motor_id}) mode={mode} dir={direction} speed={STEPS_PER_SEC} rotations={rotations:.2f} pulses={pulses} bytes={list(packet)}"
            )
        except Exception as exc:
            self.append_log(f"Send failed: {exc}")

    def on_close(self):
        self._disconnect()
        self.root.destroy()


def main():
    root = tk.Tk()
    CalibrationGui(root)
    root.mainloop()


if __name__ == "__main__":
    main()

