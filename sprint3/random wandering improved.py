import pygame
import math
import cv2
import numpy as np
import random
from collections import deque

# Initialize pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
icon = pygame.image.load('t8-logo.png')
pygame.display.set_icon(icon)
pygame.display.set_caption("Robot Simulation - SLAM Exploration")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)  # Color for lidar beams
RED = (255, 0, 0)  # Color for collision boundary

# Robot properties
robot_x, robot_y = 100, 100  # Initial position
robot_angle = 0  # Initial angle (facing right)
robot_speed = 0
max_speed = 5
collision_radius = 25  # Radius of the circular collision boundary (smaller robot)
obstacle_buffer = 5  # Buffer distance to maintain from walls (reduced for closer wall following)

# Triangle dimensions (Smaller robot)
triangle_base = 15  # Robot base width (smaller)
triangle_height = 25  # Robot height (from base to tip, smaller)

# Walls (Irregular Floor Plan Layout with Doorways)
walls = [
    # Outer walls
    ((50, 50), (750, 50)),  # Top wall
    ((50, 50), (50, 550)),  # Left wall
    ((750, 50), (750, 550)),  # Right wall
    ((50, 550), (750, 550)),  # Bottom wall

    # Internal walls with doorways
    ((200, 50), (200, 150)),  # Left vertical wall (before doorway)
    ((200, 250), (200, 400)),  # Left vertical wall (after doorway)
    ((200, 400), (400, 400)),  # Bottom-left horizontal wall
    ((400, 400), (400, 250)),  # Vertical wall (before doorway)
    ((400, 150), (400, 50)),   # Vertical wall (after doorway)
    ((400, 150), (600, 150)),  # Top-middle horizontal wall
    ((600, 150), (600, 250)),  # Vertical wall (before doorway)
    ((600, 400), (600, 550)),  # Right vertical wall (after doorway)
    ((600, 400), (750, 400)),  # Bottom-right horizontal wall
]

# SLAM-related variables
map_size = (750, 850)
slam_map = np.full(map_size, 255, dtype=np.uint8)  # Initialize the map as unexplored (white)
slam_map.fill(255)  # Initialize the map as white
visited_map = np.zeros(map_size, dtype=np.uint8)  # Map to track visited locations

# Lidar sensor simulation
lidar_range = 100  # Lidar range in pixels
lidar_angle_step = 10  # Angle step for lidar rays

# Clock
clock = pygame.time.Clock()
FPS = 60

# Stuck detection
position_history = deque(maxlen=50)  # Keep track of the last 50 positions
stuck_threshold = 1  # Threshold distance to consider as "stuck" (more conservative)
grace_period = 100  # Number of frames before enabling stuck detection
frame_count = 0
wall_following_mode = False  # Whether the robot is in wall-following mode
backtracking_mode = False  # Whether the robot is in backtracking mode
backtrack_stack = []  # Stack to keep track of the path for backtracking
path_history = []  # List to keep track of the full path history

# Spin timeout to prevent endless spinning
spin_timeout = 100  # Number of frames allowed for spinning
spin_counter = 0  # Counter to track spinning frames

# Random movement to avoid stagnation
random_movement_interval = 300  # Number of frames between random movements
random_movement_counter = 0

def draw_robot(x, y, angle):
    # Calculate the points of the isosceles triangle
    tip = (x + triangle_height * math.cos(math.radians(angle)),
           y + triangle_height * math.sin(math.radians(angle)))
    base_left = (x + triangle_base / 2 * math.cos(math.radians(angle + 90)),
                 y + triangle_base / 2 * math.sin(math.radians(angle + 90)))
    base_right = (x + triangle_base / 2 * math.cos(math.radians(angle - 90)),
                  y + triangle_base / 2 * math.sin(math.radians(angle - 90)))
    pygame.draw.polygon(screen, BLUE, [tip, base_left, base_right])
    # Draw collision boundary
    pygame.draw.circle(screen, RED, (int(x), int(y)), collision_radius, 1)

def draw_walls():
    for wall in walls:
        pygame.draw.line(screen, BLACK, wall[0], wall[1], 10)

def check_collision(x, y):
    for wall in walls:
        wall_start, wall_end = wall
        dist = distance_to_line_segment((x, y), wall_start, wall_end)
        if dist < collision_radius + obstacle_buffer:
            return True
    return False

def distance_to_line_segment(point, start, end):
    px, py = point
    sx, sy = start
    ex, ey = end
    line_len_sq = (ex - sx) ** 2 + (ey - sy) ** 2
    if line_len_sq == 0:
        return math.hypot(px - sx, py - sy)
    t = max(0, min(1, ((px - sx) * (ex - sx) + (py - sy) * (ey - sy)) / line_len_sq))
    proj_x = sx + t * (ex - sx)
    proj_y = sy + t * (ey - sy)
    return math.hypot(px - proj_x, py - proj_y)

def update_slam_map(x, y):
    if 0 <= int(x) < map_size[1] and 0 <= int(y) < map_size[0]:
        cv2.circle(slam_map, (int(x), int(y)), 3, 0, -1)
        visited_map[int(y), int(x)] = 1  # Mark the position as visited

def lidar_scan(x, y, angle):
    for lidar_angle in range(0, 360, lidar_angle_step):
        scan_angle = angle + lidar_angle
        for distance in range(0, lidar_range, 5):
            scan_x = x + distance * math.cos(math.radians(scan_angle))
            scan_y = y + distance * math.sin(math.radians(scan_angle))
            if scan_x < 0 or scan_x >= map_size[1] or scan_y < 0 or scan_y >= map_size[0]:
                break
            if check_collision(scan_x, scan_y):
                # Mark detected wall on SLAM map
                cv2.circle(slam_map, (int(scan_x), int(scan_y)), 3, 0, -1)
                break
            # Draw lidar beam in pygame
            pygame.draw.circle(screen, GREEN, (int(scan_x), int(scan_y)), 2)
            # Mark scanned area on SLAM map
            cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, 128, -1)

# Main game loop
running = True
while running:
    screen.fill(WHITE)
    frame_count += 1
    random_movement_counter += 1

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Perform lidar scan
    lidar_scan(robot_x, robot_y, robot_angle)

    # Wandering behavior
    new_x = robot_x + max_speed * math.cos(math.radians(robot_angle))
    new_y = robot_y + max_speed * math.sin(math.radians(robot_angle))
    if not check_collision(new_x, new_y):
        robot_x, robot_y = new_x, new_y
    else:
        robot_angle = (robot_angle + random.randint(-45, 45)) % 360

    # Add random movement to avoid stagnation
    if random_movement_counter > random_movement_interval:
        robot_angle = (robot_angle + random.randint(-45, 45)) % 360
        random_movement_counter = 0

    # Update SLAM map with current robot position
    update_slam_map(robot_x, robot_y)

    # Draw walls and robot
    draw_walls()
    draw_robot(robot_x, robot_y, robot_angle)

    # Update the display
    pygame.display.flip()

    # Show SLAM map using OpenCV
    cv2.imshow('SLAM Map', slam_map)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False

    # Cap the frame rate
    clock.tick(FPS)

# Quit pygame and OpenCV
pygame.quit()
cv2.destroyAllWindows()