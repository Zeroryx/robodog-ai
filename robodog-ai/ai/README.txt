# YOLOv8 Computer Vision Project - Team Collaboration Guide

This document provides the complete operational protocol for team members to initialize the local environment, synchronize dependencies, and execute real-time inference.

## 1. System Requirements & Architecture
This project is built with hardware-agnostic execution in mind. The system will automatically utilize available hardware acceleration or fallback to standard processing.

* **Target Directory:** `C:\VsCode\Binus\YOLO26_Project` (Ensure you are in the correct root).
* **Environment Architecture:** Python Virtual Environment (venv).
* **Recommended Hardware (High FPS):** Dedicated NVIDIA GPU (CUDA 11.8+ supported).
* **Minimum Hardware (Low FPS):** Standard Multi-core CPU (Intel/AMD/Apple Silicon). The script will automatically default to CPU processing if no NVIDIA GPU is detected.

## 2. Setup and Execution Protocol

### Step 1: Synchronize the Repository
Open your terminal and pull the latest updates from the main branch to ensure you have the updated script architectures:

```bash
git pull origin main
```

### Step 2: Initialize Virtual Environment
Create an isolated Python environment to manage project libraries cleanly without global conflicts:

```bash
python -m venv venv
```

### Step 3: Activate Virtual Environment
Activate the isolated space before installing any packages:

```bash
.\venv\Scripts\activate
```

*Note for Windows Users: If PowerShell returns a script execution error, run the following authorization command first, type `Y`, press Enter, and then re-run the activation command:*
```powershell
Set-ExecutionPolicy Unrestricted -Scope CurrentUser
```

*Validation: Ensure the green `(venv)` prefix is visible on the far left of your terminal prompt.*

### Step 4: Install Core Dependencies
Install the required framework libraries into your active virtual environment:

```bash
pip install ultralytics roboflow opencv-python
```

### Step 5: Place the Model Weights
Download the trained weights file `model_blokm_v1.pt` shared via the WhatsApp group. Place this exact file directly into the project root directory. Do not rename the file, as the script looks for this specific name.

### Step 6: Inference Script Architecture (test_model.py)
Ensure your `test_model.py` file contains the correct configuration for webcam stream processing:

```python
from ultralytics import YOLO

# Initialize the YOLO architecture and load custom weight matrices
model = YOLO("model_blokm_v1.pt")

# Execute continuous frame-by-frame real-time object detection
# source=0 targets the primary integrated webcam hardware
# show=True forces the system to render a visual output window
# save=False disables frame caching to protect local SSD storage
results = model.predict(source=0, show=True, save=False)
```

### Step 7: Launch the Inference Script
Run the computer vision pipeline using the primary webcam feed:

```bash
python test_model.py
```

*Note: Allow a few seconds for the system to allocate memory and load the model parameters. If you are running on a CPU, the initialization will take slightly longer and the video feed may experience lower framerates compared to a GPU.*

## 3. Safe Hardware Termination Protocol
To prevent hardware resource locking or background process leaks, stop the execution loop using one of these methods:

* Method Alpha: Click on the active webcam stream window and press the `q` key on your keyboard to trigger a clean exit loop.

