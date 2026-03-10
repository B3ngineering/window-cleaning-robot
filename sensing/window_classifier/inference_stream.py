import cv2
from ultralytics import YOLO
import time

# Load trained model
model = YOLO("runs/classify/train3/weights/best.pt")

# Open camera
cap = cv2.VideoCapture(1)

# Optimize camera settings
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FPS, 30)

print("Camera resolution:",
      cap.get(cv2.CAP_PROP_FRAME_WIDTH),
      "x",
      cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Run inference (no saving, no display inside YOLO)
    results = model(frame, verbose=False)

    r = results[0]
    predicted = r.names[r.probs.top1]
    confidence = float(r.probs.top1conf)

    if predicted == "dirty" and confidence < 0.7:
        predicted = "clean"
        confidence = 1 - confidence  # optional: flip confidence for display

    # FPS calculation
    current_time = time.time()
    fps = 1 / (current_time - prev_time) if prev_time != 0 else 0
    prev_time = current_time

    # Draw text on frame
    text = f"{predicted} ({confidence:.2f}) | FPS: {fps:.1f}"
    cv2.putText(frame, text, (20, 40),
                cv2.FONT_HERSHEY_SIMPLEX,
                1, (0, 255, 0), 2)

    cv2.imshow("Window Classifier", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
