# config/settings.py


# PACKAGE LIMITS
MAX_PACKAGE_LENGTH_CM = 15
MAX_PACKAGE_WIDTH_CM  = 15
MAX_PACKAGE_HEIGHT_CM = 15
MAX_PACKAGE_WEIGHT_KG = 2.0


# GRID MAP
GRID_CELL_SIZE_CM = 30        # how many real cm each grid cell represents
GRID_ROWS         = 20        # total rows in one floor map
GRID_COLS         = 30        # total columns in one floor map


# BASE STATION
BASE_STATION_FLOOR    = 1
BASE_STATION_GRID_ROW = 0
BASE_STATION_GRID_COL = 0



# ROBOT BEHAVIOR
COUNTDOWN_SECONDS          = 10
ROBOT_SPEED_CELLS_PER_SEC  = 1.0
OBSTACLE_DECAY_SECONDS     = 5.0   # how long before a dynamic obstacle disappears from map
REPLAN_COOLDOWN_SECONDS    = 1.0   # minimum time between A* replanning calls



# YOLO / PERCEPTION
YOLO_MODEL_PATH        = "data/models/yolov8n.pt"
YOLO_CONFIDENCE_THRESH = 0.5
YOLO_TARGET_CLASSES    = [0]       # 0 = person in COCO dataset
DETECTION_FPS          = 5         # how many times per second YOLO runs



# CAMPUS ROOMS
# harus punya denah syahdan dulu
CAMPUS_ROOMS = {
    "101": {"floor": 1, "grid_pos": (18, 10), "label": "Classroom 101"},
    "102": {"floor": 1, "grid_pos": (18, 20), "label": "Classroom 102"},
    "201": {"floor": 2, "grid_pos": (18, 10), "label": "Classroom 201"},
    "LAB1": {"floor": 2, "grid_pos": (5, 25),  "label": "Computer Lab 1"},
    # add the rest of BINUS Syahdan rooms here
}

FLOOR_COUNT = 2


# SIMULATION DISPLAY

SIM_WINDOW_TITLE   = "BINUS Robotic Dog - Delivery Simulation"
SIM_CELL_PIXEL_SIZE = 30          # pixels per grid cell in the visualization
SIM_FPS             = 30