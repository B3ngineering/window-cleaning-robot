import queue
import threading
import time
import serial


class SerialStream:
    def __init__(self, port, baudrate=115200, timeout=0.05, event_queue=None):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.event_queue = event_queue

        self.connection = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None
        self.results_queue = queue.Queue()

    def start(self):
        if self.thread and self.thread.is_alive():
            return self

        self.connection = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
        )

        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self.stop_event.set()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        if self.connection is not None and self.connection.is_open:
            self.connection.close()
            self.connection = None

    def read(self):
        with self.lock:
            return None if self.latest is None else dict(self.latest)

    def get_next(self, timeout=None):
        try:
            return self.results_queue.get(timeout=timeout)
        except queue.Empty:
            return None

    def _run(self):
        while not self.stop_event.is_set():
            raw = self.connection.readline()
            if not raw:
                continue

            message = raw.decode("utf-8", errors="replace").strip()
            if not message:
                continue

            with self.lock:
                self.latest = {
                    "value": message,
                    "timestamp": time.time(),
                }

            payload = dict(self.latest)
            self.results_queue.put(payload)
            if self.event_queue is not None:
                self.event_queue.put(
                    {
                        "source": "serial",
                        "data": payload,
                    }
                )