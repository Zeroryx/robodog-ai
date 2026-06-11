# 🐾 AI Robotic Dog — Indoor Delivery System
### BINUS University | Syahdan Campus | Software Architecture Project

> An AI-powered robotic dog simulation for autonomous indoor package delivery.
> Built with Python, A* pathfinding, YOLOv8 obstacle detection, and a finite state machine delivery workflow.

---

## 📋 Table of Contents

1. [Project Overview](#-project-overview)
2. [Project Objectives](#-project-objectives)
3. [What This Project Does](#-what-this-project-does)
4. [System Architecture](#-system-architecture)
5. [Features](#-features)
6. [Project Structure](#-project-structure)
7. [Requirements](#-requirements)
8. [Installation Guide](#-installation-guide)
9. [Running the Project](#-running-the-project)
10. [Configuration](#-configuration)
11. [Delivery Workflow](#-delivery-workflow)
12. [Git Workflow Guide](#-git-workflow-guide)
13. [Development Workflow](#-development-workflow)
14. [Troubleshooting](#-troubleshooting)
15. [Contributing Guidelines](#-contributing-guidelines)
16. [Future Improvements](#-future-improvements)
17. [Team](#-team)
18. [License](#-license)

---

## 📌 Project Overview

This project is the complete software architecture for an **AI-powered robotic dog** designed to perform autonomous indoor package delivery inside the **BINUS University Syahdan Campus** building. The system is built entirely in Python and runs as a **simulation first** — no physical robot hardware is required to develop, test, or demonstrate the software.

The robot navigates multi-floor indoor environments, avoids dynamic obstacles using real-time YOLOv8 object detection, validates packages against size and weight limits, and manages the full delivery lifecycle from sender QR verification to receiver confirmation using a Finite State Machine.

This project is developed as a university software architecture project. All navigation, pathfinding, perception, and delivery logic is designed to be hardware-agnostic — meaning the same codebase will work on a real robotic dog by swapping only the hardware interface layer.

---

## 🎯 Project Objectives

- Design and implement a complete software system for an autonomous indoor delivery robot
- Demonstrate A\* pathfinding on a grid map representing a real building layout
- Integrate YOLOv8 object detection for real-time dynamic obstacle avoidance
- Implement a robust Finite State Machine to manage the full delivery workflow
- Validate package size and weight before accepting a delivery
- Verify sender and receiver identity using QR codes
- Support multi-floor navigation within a known building environment
- Build a simulation that visually demonstrates robot movement and obstacle avoidance
- Produce software that is ready for hardware integration in a future phase

---

## 🤖 What This Project Does

When the application runs, it launches a simulation of the BINUS Syahdan Campus building. A robotic dog starts at a fixed **base station** on Floor 1 and waits for a delivery request.

The full delivery process works as follows:

1. A sender interacts with the UI and scans a QR code to authenticate
2. The robot verifies the QR code and grants access
3. The sender selects the destination floor and room number
4. The sender enters the package dimensions (length, width, height in cm) and weight (kg)
5. The system validates the package — maximum 15×15×15 cm and 2 kg
6. If valid, the robot accepts the package and a countdown begins
7. After the countdown, the robot begins navigating to the destination using A\* pathfinding
8. During navigation, YOLOv8 continuously scans the camera feed for obstacles
9. If an obstacle (e.g., a person) is detected in the robot's path, the grid map is updated and A\* recalculates the route in real time
10. The robot arrives at the destination room
11. The receiver scans their QR code to confirm identity
12. The package is handed over and removed from the robot
13. The robot autonomously returns to the base station
14. The system returns to idle state, ready for the next delivery

All movement, obstacle detection, and state transitions are visualized in real time using Matplotlib.

---

## 🏗 System Architecture

The project is divided into eight independent modules. Each module has a single clear responsibility and communicates with others through defined interfaces. This design means any module can be replaced (e.g., swapping simulation for real hardware) without touching the others.

┌─────────────────────────────────────────────────────────┐
│                        main.py                          │
│                 Entry point — starts app                │
└────────────────────────┬────────────────────────────────┘
│
┌──────────▼──────────┐
│      core/          │
│  robot.py           │  ← Central coordinator
│  state_machine.py   │  ← FSM: controls all states
└──┬───┬───┬───┬──┬──┘
│   │   │   │  │
┌─────────┘   │   │   │  └──────────────┐
│             │   │   │                 │
┌────▼────┐  ┌─────▼─┐ │ ┌▼──────────┐ ┌───▼────┐
│navigation│  │delivery│ │ │perception │ │interface│
│grid_map  │  │manager │ │ │detector   │ │   _UI   │
│astar     │  │qr_sys  │ │ │ob_tracker │ │screens  │
│map_mgr   │  │pkg_val │ │ └─────┬─────┘ └──────── ┘
│path_exec │  └────────┘ │       │
│obstacle  │             │       │ obstacle positions
└────┬─────┘   ┌─────────┘       │
│         │                 │
┌────▼─────────▼─────────────────▼────┐
│            hardware/                 │
│   motor_controller.py (abstract)     │
│   sensor_interface.py (abstract)     │
└────────────────┬─────────────────────┘
│
┌────────▼────────┐
│   simulation/   │
│   sim_robot.py  │  ← Implements hardware interface
│   sim_env.py    │  ← Visual rendering
└─────────────────┘

**Data flow summary:**

- `config/` feeds constants into every module
- `core/robot.py` holds references to all subsystems and coordinates them
- `core/state_machine.py` is the single source of truth for what the robot is currently doing
- `perception/` detects obstacles and passes grid coordinates to `navigation/`
- `navigation/` plans and executes the path, updates the grid when obstacles are detected
- `delivery/` manages the business logic of the delivery sequence
- `hardware/` defines abstract interfaces — `simulation/` provides the concrete implementations
- `interface_UI/` reads state and displays it; never controls the robot directly

---

## ✅ Features

### Currently Implemented
- Grid-based map representation of BINUS Syahdan Campus floor plans
- A\* pathfinding algorithm for shortest-path route planning
- Robot movement simulation with real-time Matplotlib visualization
- Dynamic goal setting via mouse click or UI input
- Finite State Machine with 11 states managing the full delivery lifecycle
- Package dimension and weight validation
- QR code generation and mock verification system
- Multi-floor map management with floor-switching logic
- Path executor that moves the robot step by step along the planned path
- Dynamic obstacle handler that updates the grid and triggers A\* replanning
- Centralized configuration via `config/settings.py`
- Campus room database via `config/campuslayout.py`
- Logging system via `utils/logger.py`

### In Progress
- YOLOv8 real-time obstacle detection integration
- Full UI screen flow (sender QR → form → countdown → live status)
- End-to-end delivery simulation with all 16 workflow steps

### Planned (Future Phase)
- SLAM (Simultaneous Localization and Mapping) with LiDAR
- Real camera integration
- Elevator interaction for floor transitions
- Guide Dog mode
- Patrol mode
- Delivery scheduling
- Fleet management for multiple robots
- Voice command integration (Whisper ASR)
- REST API for remote delivery requests

---

## 📁 Project Structure
robodog-ai/
│
├── main.py                        # Entry point. Starts the app. Keep this file short.
│
├── requirements.txt               # All Python dependencies
├── README.md                      # This file
├── .gitignore                     # Files and folders excluded from Git
│
├── config/                        # Global configuration — edit these to change robot behavior
│   ├── init.py
│   ├── settings.py                # All constants: limits, speeds, grid size, YOLO config
│   └── campuslayout.py            # Room database: maps room numbers to floor and grid coordinates
│
├── core/                          # The robot's brain — central coordination
│   ├── init.py
│   ├── robot.py                   # Main Robot class. Holds all subsystems. Coordinates everything.
│   └── state_machine.py           # Finite State Machine. Defines all 11 robot states and valid transitions.
│
├── navigation/                    # Everything about moving through the building
│   ├── init.py
│   ├── gridmap.py                 # 2D grid representation of a single floor
│   ├── astar.py                   # A* pathfinding algorithm (pure logic, no hardware)
│   ├── mapmanager.py              # Loads floor maps, manages floor switching, knows staircase positions
│   ├── pathexecutor.py            # Takes A* output and commands hardware to follow the path
│   └── obstacle.py                # Receives obstacle positions, updates grid, triggers replanning
│
├── perception/                    # Everything the robot sees
│   ├── init.py
│   ├── detector.py                # YOLOv8 wrapper. Reads camera, runs inference, outputs bounding boxes.
│   └── obstacle_tracker.py        # Converts pixel detections to grid coordinates. Feeds obstacle.py.
│
├── delivery/                      # Delivery business logic
│   ├── init.py
│   ├── delivery_manager.py        # Orchestrates the full delivery sequence
│   ├── package_validator.py       # Validates dimensions and weight against limits in settings.py
│   └── qr_system.py               # Generates, stores, and verifies QR codes for sender and receiver
│
├── hardware/                      # Hardware abstraction layer
│   ├── init.py
│   ├── motor_controller.py        # Abstract interface: move_forward, turn_left, turn_right, stop
│   └── sensor_interface.py        # Abstract interface: get_camera_frame, get_distance_front
│
├── simulation/                    # Fake hardware for development (implements hardware/ interfaces)
│   ├── init.py
│   ├── sim_robot.py               # Simulated robot. Implements motor_controller. Animates movement.
│   └── sim_environment.py         # Renders the campus grid, robot, path, and obstacles in Matplotlib.
│
├── modes/                         # Robot operating modes
│   ├── init.py
│   ├── base_mode.py               # Abstract base class: start(), stop(), on_event()
│   └── delivery_mode.py           # Delivery mode implementation
│
├── interface_UI/                  # User interface
│   ├── init.py
│   └── screens/                   # Individual UI screens
│       ├── init.py
│       ├── home_screen.py         # Starting screen
│       ├── qr_screen.py           # QR code input/scan screen
│       ├── delivery_form.py       # Floor selection, room selection, package dimensions
│       ├── countdown_screen.py    # Countdown before navigation begins
│       └── status_screen.py       # Live delivery status and robot position
│
├── utils/                         # Shared utilities used across all modules
│   ├── init.py
│   └── logger.py                  # Centralized logger. All modules import from here.
│
├── data/
│   ├── maps/                      # Floor map files (floor1.json, floor2.json, etc.)
│   └── models/                    # YOLOv8 model weights (yolov8n.pt — downloaded separately)
│
└── ztest/                         # Automated tests
├── test_astar.py
├── test_package_validator.py
└── test_state_machine.py


### Files delegation

| File | Who edits it | Why |
|---|---|---|
| `config/settings.py` | Everyone | Change limits, speeds, grid dimensions |
| `config/campuslayout.py` | Navigation Dev | Add or update room coordinates |
| `core/state_machine.py` | Architect / Lead | Add new states or transitions |
| `navigation/astar.py` | Navigation Dev | Improve pathfinding logic |
| `navigation/gridmap.py` | Navigation Dev | Change grid cell behavior |
| `perception/detector.py` | AI/Vision Dev | Tune YOLO confidence, add new detection classes |
| `delivery/package_validator.py` | Delivery Dev | Change validation rules |
| `delivery/qr_system.py` | Delivery Dev | Implement real QR scanning |
| `interface_UI/screens/*.py` | UI Dev | Build and style the UI screens |
| `simulation/sim_environment.py` | Simulation Dev | Improve visualization |

### Files you should NOT edit unless you fully understand the impact

| File | Reason |
|---|---|
| `core/robot.py` | Central coordinator — changes affect every module |
| `hardware/motor_controller.py` | Abstract interface — changing it breaks simulation AND future real robot |
| `hardware/sensor_interface.py` | Same reason as above |
| `navigation/mapmanager.py` | Changing floor-switching logic affects multi-floor pathfinding |

---

## 📦 Requirements

### System Requirements

| Requirement | Minimum |
|---|---|
| Operating System | Windows 10/11, macOS 12+, or Ubuntu 20.04+ |
| Python Version | 3.10 or higher (3.11 recommended) |
| RAM | 8 GB minimum (16 GB recommended for YOLO) |
| Storage | 5 GB free (for dependencies and YOLO model) |
| GPU | Optional but recommended for YOLO inference |

### Python Dependencies

All dependencies are listed in `requirements.txt`. Key packages:

| Package | Version | Purpose |
|---|---|---|
| `torch` | ≥2.0.0 | PyTorch — required backend for YOLOv8 |
| `torchvision` | ≥0.15.0 | PyTorch vision utilities |
| `ultralytics` | ≥8.0.0 | YOLOv8 object detection framework |
| `opencv-python` | ≥4.8.0 | Image processing and camera handling |
| `numpy` | ≥1.24.0 | Array math, grid operations |
| `matplotlib` | ≥3.7.0 | Simulation visualization |
| `Pillow` | ≥9.5.0 | Image handling |
| `qrcode` | ≥7.4.0 | QR code generation |
| `pyzbar` | ≥0.1.9 | QR code decoding |
| `pydantic` | ≥2.0.0 | Data validation |
| `scipy` | ≥1.11.0 | Math utilities (prepared for SLAM) |
| `pytest` | ≥7.4.0 | Automated testing |

---

## 🛠 Installation Guide

Follow every step in order. Do not skip steps.

### Step 1 — Install Python

Download and install Python 3.11 from the official website:
👉 https://www.python.org/downloads/

During installation on Windows, check the box that says **"Add Python to PATH"** before clicking Install.

Verify the installation by opening a terminal and running:

```bash
python --version
```

Expected output:
Python 3.11.x

### Step 2 — Clone the Repository

Open a terminal (PowerShell on Windows, Terminal on macOS/Linux).

Navigate to the folder where you want to store the project. Replace `YourFolder` with your chosen path:

```bash
cd C:\Users\YourName\Documents\YourFolder
```

Clone the repository from GitHub:

```bash
git clone https://github.com/YOUR_TEAM_USERNAME/robodog-ai.git
```

> Replace `YOUR_TEAM_USERNAME` with the actual GitHub username or organization where the repository is hosted.

This command downloads the entire project to your computer into a new folder called `robodog-ai`.

Enter the project folder:

```bash
cd robodog-ai
```

---

### Step 3 — Create a Virtual Environment

A virtual environment is an isolated Python environment. It keeps this project's dependencies separate from other Python projects on your computer. **Every team member must create their own virtual environment — it is not shared through Git.**

Create the virtual environment inside the project folder:

```bash
python -m venv venv
```

This creates a folder called `venv/` in the project. This folder is excluded from Git by `.gitignore` — you should never commit it.

---

### Step 4 — Activate the Virtual Environment

You must activate the virtual environment every time you open a new terminal to work on this project.

**On Windows (PowerShell):**
```powershell
venv\Scripts\Activate.ps1
```

If you see an error about execution policy, run this first:
```powershell
Set-ExecutionPolicy -Scope Process -ExecutionPolicy RemoteSigned
```
Then try the activation command again.

**On Windows (Command Prompt):**
```cmd
venv\Scripts\activate.bat
```

**On macOS / Linux:**
```bash
source venv/bin/activate
```

When activation succeeds, your terminal prompt will change to show `(venv)` at the beginning:
(venv) PS C:\Users\YourName\Documents\robodog-ai>

---

### Step 5 — Install PyTorch (Must Be Done Before Other Packages)

PyTorch must be installed separately before everything else because it requires a specific installation URL depending on whether your machine has a GPU.

**If your laptop does NOT have an NVIDIA GPU (most laptops):**
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

**If your laptop HAS an NVIDIA GPU (CUDA 11.8):**
```bash
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu118
```

Not sure which to use? Use the CPU version. It works on all machines.

Verify PyTorch installed correctly:
```bash
python -c "import torch; print(torch.__version__)"
```

---

### Step 6 — Install All Other Dependencies

With PyTorch already installed and the virtual environment active, install all remaining packages:

```bash
pip install -r requirements.txt
```

This command reads every line in `requirements.txt` and installs all listed packages. This may take 5–15 minutes depending on your internet speed.

Verify the installation by checking a few packages:
```bash
pip show ultralytics
pip show numpy
pip show matplotlib
```

---

### Step 7 — Download the YOLO Model Weights

YOLOv8 requires a pre-trained model file (`.pt`) to function. This file is **not included in the repository** because it is too large for Git. Download it by running the following command from the project root:

```bash
python -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

This automatically downloads `yolov8n.pt` (~6 MB) to the current directory.

Move the downloaded file to the correct location:

**On Windows:**
```powershell
move yolov8n.pt data\models\yolov8n.pt
```

**On macOS / Linux:**
```bash
mv yolov8n.pt data/models/yolov8n.pt
```

Confirm the file is in the right place:
robodog-ai/
└── data/
└── models/
└── yolov8n.pt   ✅

---

### Step 8 — Windows Only: Fix pyzbar (QR Decoding)

On Windows, `pyzbar` requires an additional DLL file. If you encounter an error when using QR code scanning, run:

```bash
pip install pyzbar[scripts]
```

If the error persists, download the ZBar DLL from http://zbar.sourceforge.net and place `libzbar-64.dll` in the project root folder.

---

## ▶️ Running the Project

Make sure your virtual environment is activated (you should see `(venv)` in your terminal prompt).

From the project root folder (`robodog-ai/`), run:

```bash
python main.py
```

### What happens when you run main.py

1. Python loads all configuration from `config/settings.py` and `config/campuslayout.py`
2. The `Robot` object is initialized in `core/robot.py`, which sets up all subsystems
3. The State Machine starts in the `IDLE` state
4. The simulation environment launches and displays the BINUS Syahdan campus grid
5. The robot appears at the base station (Floor 1, grid position defined in `settings.py`)
6. The UI home screen opens, waiting for a delivery request

You should see a Matplotlib window showing the floor map and the robot's starting position.

### Running the tests

To verify that core logic is working correctly:

```bash
pytest ztest/
```

To run a specific test file:

```bash
pytest ztest/test_astar.py
pytest ztest/test_package_validator.py
pytest ztest/test_state_machine.py
```

To run tests with a coverage report:

```bash
pytest ztest/ --cov=. --cov-report=term-missing
```

---

## ⚙️ Configuration

All configurable values are stored in `config/settings.py`. **Do not hardcode values in any other file.** If you need a constant in your module, import it from here.

### Package Limits

```python
MAX_PACKAGE_LENGTH_CM = 15
MAX_PACKAGE_WIDTH_CM  = 15
MAX_PACKAGE_HEIGHT_CM = 15
MAX_PACKAGE_WEIGHT_KG = 2.0
```

Changing these values here automatically updates validation in `delivery/package_validator.py` and the UI form labels.

### Grid Map Settings

```python
GRID_CELL_SIZE_CM = 30    # Real-world size of one grid cell
GRID_ROWS         = 20    # Number of rows in the floor grid
GRID_COLS         = 30    # Number of columns in the floor grid
```

If you update these, you must also regenerate the floor map files in `data/maps/`.

### Base Station

```python
BASE_STATION_FLOOR    = 1
BASE_STATION_GRID_ROW = 0
BASE_STATION_GRID_COL = 0
```

This is where the robot starts and returns after every delivery.

### Robot Behavior

```python
COUNTDOWN_SECONDS         = 10
ROBOT_SPEED_CELLS_PER_SEC = 1.0
OBSTACLE_DECAY_SECONDS    = 5.0
REPLAN_COOLDOWN_SECONDS   = 1.0
```

### YOLO Settings

```python
YOLO_MODEL_PATH        = "data/models/yolov8n.pt"
YOLO_CONFIDENCE_THRESH = 0.5
YOLO_TARGET_CLASSES    = [0]    # 0 = person in COCO dataset
DETECTION_FPS          = 5
```

### Adding a New Room

Open `config/campuslayout.py` and add an entry to `CAMPUS_ROOMS`:

```python
CAMPUS_ROOMS = {
    "101": {"floor": 1, "grid_pos": (18, 10), "label": "Classroom 101"},
    "NEW_ROOM": {"floor": 2, "grid_pos": (5, 14), "label": "Your Room Name"},
}
```

The room immediately becomes available in the floor/room selection UI and as a navigation target.

---

## 🚚 Delivery Workflow

This section explains exactly what the robot does from start to finish during one delivery.

User Action                    Robot State
─────────────────────────────────────────────────────────
App starts                  →  IDLE
User scans QR code          →  VERIFYING_SENDER_QR
├── QR invalid            →  IDLE (show error)
└── QR valid              →  FORM_INPUT
User fills delivery form    →  FORM_INPUT
├── Package too big/heavy →  FORM_INPUT (show error)
└── Package valid         →  PACKAGE_ACCEPTED
Package placed on robot     →  COUNTDOWN (10 second timer)
Countdown ends              →  NAVIGATING
├── Obstacle detected     →  (stays NAVIGATING, path recalculated)
├── Path fully blocked    →  ERROR
└── Destination reached   →  AT_DESTINATION
Receiver scans QR           →  VERIFYING_RECEIVER_QR
├── QR invalid            →  AT_DESTINATION (try again)
└── QR valid              →  DELIVERING
Package removed             →  RETURNING
Base station reached        →  IDLE

### State Descriptions

| State | What is happening |
|---|---|
| `IDLE` | Robot is at base station, waiting for a delivery request |
| `VERIFYING_SENDER_QR` | Robot is checking the sender's QR token against the database |
| `FORM_INPUT` | User is entering destination and package details |
| `PACKAGE_ACCEPTED` | Package passed validation, robot is ready to receive it |
| `COUNTDOWN` | 10-second countdown before robot begins moving |
| `NAVIGATING` | Robot is actively moving toward the destination |
| `AT_DESTINATION` | Robot has arrived, waiting for receiver |
| `VERIFYING_RECEIVER_QR` | Robot is checking the receiver's QR token |
| `DELIVERING` | QR verified, package is being handed to receiver |
| `RETURNING` | Robot is navigating back to the base station |
| `ERROR` | Navigation failure or critical error, waiting for reset |

---

## 🔁 Git Workflow Guide

This section is for all team members. Follow this workflow every time you work on the project to avoid merge conflicts and broken code.

### First-time setup

Configure your Git identity once on your machine:

```bash
git config --global user.name "Your Name"
git config --global user.email "your.email@binus.ac.id"
```

---

### Daily workflow — do this every time you sit down to code

**Step 1 — Pull the latest changes from GitHub**

Before writing any code, always get the latest version from your teammates:

```bash
git pull origin main
```

`git pull` downloads and merges any changes your teammates pushed since the last time you pulled. If you skip this, you risk writing code on an outdated version and creating conflicts later.

---

**Step 2 — Check what has changed**

After writing code, check which files you modified:

```bash
git status
```

This shows three categories:
- **Changes to be committed** — files staged and ready to commit
- **Changes not staged for commit** — files you modified but haven't staged yet
- **Untracked files** — new files Git doesn't know about yet

---

**Step 3 — Stage your changes**

To stage a specific file:

```bash
git add config/settings.py
```

To stage all changed files at once:

```bash
git add .
```

> Avoid `git add .` if you have temporary or test files you don't want to commit. Stage files individually when in doubt.

---

**Step 4 — Commit your changes**

A commit saves a snapshot of your staged changes with a message describing what you did:

```bash
git commit -m "Add package weight validation to package_validator.py"
```

**Good commit message rules:**
- Start with a verb: Add, Fix, Update, Remove, Refactor, Implement
- Be specific — say what changed and where
- Keep it under 72 characters

**Good examples:**
Add MAX_PACKAGE_WEIGHT_KG constant to settings.py
Fix A* failing when start equals goal position
Implement QR token generation in qr_system.py
Refactor state_machine transitions to use enum guards

**Bad examples:**
update
fix bug
changes
wip

---

**Step 5 — Push your changes to GitHub**

Send your committed changes to the shared repository:

```bash
git push origin main
```

---

### Full daily workflow summary

```bash
# 1. Get latest code from teammates
git pull origin main

# 2. Write your code...

# 3. Check what you changed
git status

# 4. Stage your changes
git add navigation/astar.py

# 5. Commit with a clear message
git commit -m "Improve A* heuristic to support diagonal movement"

# 6. Push to GitHub
git push origin main
```

---

### Handling merge conflicts

A merge conflict happens when two people edit the same line of the same file. Git will mark the conflict in the file like this: