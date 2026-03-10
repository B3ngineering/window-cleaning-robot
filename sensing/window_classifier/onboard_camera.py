import threading
import time

import cv2
from ultralytics import YOLO


DEFAULT_MODEL_PATH = "runs/classify/train3/weights/best.pt"


class OnboardCameraStream:
    def __init__(
        self,
        model_path=DEFAULT_MODEL_PATH,
        camera_index=1,
        dirty_threshold=0.7,
        frame_width=640,
        frame_height=480,
        fps=30,
    ):
        self.model = YOLO(model_path)
        self.camera_index = camera_index
        self.dirty_threshold = dirty_threshold
        self.frame_width = frame_width
        self.frame_height = frame_height
        self.fps = fps

        self.cap = None
        self.thread = None
        self.stop_event = threading.Event()
        self.lock = threading.Lock()
        self.latest = None

    def start(self):
        if self.thread and self.thread.is_alive():
            return self

        self.cap = cv2.VideoCapture(self.camera_index)
        self.cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_width)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_height)
        self.cap.set(cv2.CAP_PROP_FPS, self.fps)

        if not self.cap.isOpened():
            raise RuntimeError(f"Could not open camera index {self.camera_index}")

        self.stop_event.clear()
        self.thread = threading.Thread(target=self._run, daemon=True)
        self.thread.start()
        return self

    def stop(self):
        self.stop_event.set()

        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2)

        if self.cap is not None:
            self.cap.release()
            self.cap = None

    def read(self):
        with self.lock:
            return None if self.latest is None else dict(self.latest)

    def get_latest_value(self, default=None):
        result = self.read()
        if result is None:
            return default
        return result["value"]

    def set_dirty_threshold(self, threshold):
        self.dirty_threshold = threshold

    def _run(self):
        while not self.stop_event.is_set():
            ret, frame = self.cap.read()
            if not ret:
                time.sleep(0.01)
                continue

            results = self.model(frame, verbose=False)
            prediction = self._prediction_to_binary(results[0])

            with self.lock:
                self.latest = prediction

    def _prediction_to_binary(self, result):
        predicted = result.names[result.probs.top1]
        confidence = float(result.probs.top1conf)

        if predicted == "dirty" and confidence >= self.dirty_threshold:
            value = 0
            label = "dirty"
        else:
            value = 1
            label = "clean"

        return {
            "value": value,
            "label": label,
            "confidence": confidence,
            "threshold": self.dirty_threshold,
            "timestamp": time.time(),
        }


def main():
    stream = OnboardCameraStream()
    stream.start()

    try:
        last_timestamp = None
        while True:
            result = stream.read()
            if result is None or result["timestamp"] == last_timestamp:
                time.sleep(0.01)
                continue

            last_timestamp = result["timestamp"]
            print(result["value"])
    except KeyboardInterrupt:
        pass
    finally:
        stream.stop()


if __name__ == "__main__":
    main()
