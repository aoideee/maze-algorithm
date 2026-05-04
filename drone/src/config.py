# --- Network settings ---
DRONE_IP = "192.168.10.1"
DRONE_CMD_PORT = 8889
HOST_CMD_PORT = 9000
HOST_STATE_PORT = 8890
STREAM_URL = "udp://0.0.0.0:11111"

# --- Green LED detection (HSV color thresholds) ---
GREEN_HSV_LOW = (40, 100, 100)
GREEN_HSV_HIGH = (85, 255, 255)

# --- Signal confirmation settings ---
BLOB_AREA_MIN = 30
STABLE_FRAME_COUNT = 15

# --- Flight parameters ---
BATTERY_FLOOR = 25
ASCENT_HEIGHT_CM = 80
MAZE_ENTRY_CM = 280
FLIP_DIR = "f"   # valid: l, r, f, b
STEP_PAUSE = 3.0
KEEPALIVE_PERIOD = 10.0