from ultralytics import YOLO

# Load a model
model = YOLO('yolo11n.pt')

# Train the model
results = model.train(
    data='/opt/seeed/development_guide/yolo/test_dataset/detect_demo/data.yaml', 
    batch=8, epochs=100, imgsz=640, save_period=5
)
