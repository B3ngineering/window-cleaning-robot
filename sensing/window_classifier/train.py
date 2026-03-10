from ultralytics import YOLO

model = YOLO("yolo26n-cls.pt")

model.train(
    data="dataset",
    imgsz=480,  # Bump up slightly for fine detail
    epochs=100,  # More epochs for subtle features (best one for the runs so far is around 20)
    batch=16,  # Reduce if GPU memory is tight; 32 is fine if it fits
    degrees=15,  # Increase - glass can be viewed at various angles
    scale=0.3,  # Slightly more scale variation
    fliplr=0.5,  # Good
    flipud=0.3,  # Add vertical flip - glass orientation varies
    hsv_h=0.015,  # Reduce hue - lighting is important for dirt visibility
    hsv_s=0.4,  # Increase saturation variance
    hsv_v=0.4,  # Increase brightness variance - crucial for different lighting
    translate=0.1,  # Add translation - viewing position varies
    perspective=0.0001,  # Slight perspective shift
    mixup=0.1,  # Add mixup for better generalization
    patience=15,  # Increase patience for better convergence
    lr0=0.01,  # May want to tune learning rate
    save_period=10  # Save checkpoints
)

metrics = model.val()
print(metrics)

# Export to ONNX, then convert to optimized format for RPi
model.export(format="onnx")
# For RPi, also try:
# model.export(format="ncnn")  # Better for ARM CPU
# or model.export(format="tflite", int8=True)  # Quantized

#----
# https://github.com/JunjH/Visualizing-CNNs-for-monocular-depth-estimation
# try to segment foreground/background (get mask, just look at foreground)

# RCNN, bounding box classificatoins
# bounding box proposal + classification

# yolo object detection, maybe better 
# don't necessarily need multpile classes
