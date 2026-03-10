from ultralytics import YOLO
from pathlib import Path

model = YOLO("runs/classify/train4/weights/best.pt")

test_path = Path("data/testing")

# Run inference on all jpg images
results = model(str(test_path / "*.jpg"))

for r in results:
    img_name = Path(r.path).name
    predicted = r.names[r.probs.top1]
    confidence = float(r.probs.top1conf)

    print(f"{img_name} -> {predicted} ({confidence:.3f})")
