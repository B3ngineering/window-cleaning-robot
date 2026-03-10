from onboard_camera import OnboardCameraStream
import time


def main():
    stream = OnboardCameraStream(dirty_threshold=0.7)
    stream.start()

    try:
        while True:
            result = stream.read()
            if result is not None:
                print(result["value"])  # 1 = clean, 0 = dirty
            time.sleep(0.05)
    finally:
        stream.stop()


if __name__ == "__main__":
    main()
