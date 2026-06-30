from ultralytics import YOLO

# Initialize the YOLO class with your trained weights
model = YOLO("model_blokm_v1.pt")

# Execute real-time inference via default webcam
# source=0 targets the primary camera hardware
# save=False is critical to prevent storage overflow
results = model.predict(source=0, show=True, save=False)