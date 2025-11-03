import numpy as np
import matplotlib.pyplot as plt
import heapq
import serial
import time
import math

# === CONFIG ===
MAP_FILE = "lab8_map.csv"    # must exist from map_rpi_program.py
START = (10, 10)             # (row, col)
GOAL = (15, 18)
PORT = "/dev/ttyACM0"
BAUD = 9600
CELL_SIZE = 20               # cm per grid cell
MOVE_DELAY = 1.0             # seconds between commands
OBSTACLE_THRESHOLD = 0.8     # anything >= this is obstacle

# === LOAD MAP ===
grid = np.loadtxt(MAP_FILE, delimiter=",")
rows, cols = grid.shape

def heuristic(a, b):
    return math.hypot(a[0] - b[0], a[1] - b[1])

def get_neighbors(node):
    neighbors = []
    directions = [(-1,0),(1,0),(0,-1),(0,1),(-1,-1),(-1,1),(1,-1),(1,1)]
    for dr, dc in directions:
        nr, nc = node[0] + dr, node[1] + dc
        if 0 <= nr < rows and 0 <= nc < cols and grid[nr][nc] < OBSTACLE_THRESHOLD:
            neighbors.append((nr, nc))
    return neighbors

def a_star(start, goal):
    open_set = []
    heapq.heappush(open_set, (0, start))
    came_from = {}
    g_score = {start: 0}
    f_score = {start: heuristic(start, goal)}

    while open_set:
        _, current = heapq.heappop(open_set)
        if current == goal:
            path = [current]
            while current in came_from:
                current = came_from[current]
                path.append(current)
            return list(reversed(path))
        for neighbor in get_neighbors(current):
            cost = 1.0 if abs(neighbor[0]-current[0]) + abs(neighbor[1]-current[1]) == 1 else 1.414
            tentative_g = g_score[current] + cost
            if neighbor not in g_score or tentative_g < g_score[neighbor]:
                came_from[neighbor] = current
                g_score[neighbor] = tentative_g
                f_score[neighbor] = tentative_g + heuristic(neighbor, goal)
                heapq.heappush(open_set, (f_score[neighbor], neighbor))
    return None

def normalize_angle(angle):
    while angle > 180:
        angle -= 360
    while angle < -180:
        angle += 360
    return angle

def get_angle(curr, nxt):
    dy = nxt[0] - curr[0]
    dx = nxt[1] - curr[1]
    return math.degrees(math.atan2(dy, dx))

# === CONNECT TO ARDUINO ===
try:
    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)
except Exception as e:
    print(f"⚠️ Serial connection failed: {e}")
    ser = None

# === RUN A* ===
path = a_star(START, GOAL)
if path is None:
    print("❌ No path found!")
    exit()

print(f"✅ Path found with {len(path)} steps.")
print(path)

# === SEND PATH COMMANDS ===
current_angle = 0
if ser:
    for i in range(1, len(path)):
        curr = path[i-1]
        nxt = path[i]
        target_angle = get_angle(curr, nxt)
        rotate_angle = normalize_angle(target_angle - current_angle)
        current_angle = target_angle

        cmd1 = f"ROTATE {int(rotate_angle)}\n"
        cmd2 = f"MOVE {CELL_SIZE}\n"
        print(cmd1.strip(), "→", cmd2.strip())

        ser.write(cmd1.encode())
        time.sleep(MOVE_DELAY)
        ser.write(cmd2.encode())
        time.sleep(MOVE_DELAY)
    ser.close()
else:
    print("⚠️ Skipped Arduino commands (no serial).")

# === VISUALIZE PATH ===
vis = grid.copy()
for r, c in path:
    vis[r][c] = 0.4

plt.figure(figsize=(6,6))
plt.imshow(vis, cmap="coolwarm", origin="lower")
plt.scatter(START[1], START[0], c="green", marker="o", label="Start")
plt.scatter(GOAL[1], GOAL[0], c="red", marker="x", label="Goal")
plt.legend()
plt.title("Lab 9 – A* Path and Smart Car Commands")
plt.show()
