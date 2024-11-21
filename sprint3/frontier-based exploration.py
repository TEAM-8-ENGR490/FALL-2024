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
pygame.display.set_caption("Robot Simulation - Frontier-based Exploration")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)  # Color for lidar beams
RED = (255, 0, 0)  # Color for collision boundary
YELLOW = (255, 255, 0)  # Color for frontier target

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
stuck_recovery_attempts = 0
max_recovery_attempts = 5
wall_following_mode = False  # Whether the robot is in wall-following mode
paused_frontier = None  # Keep track of the paused frontier target when stuck
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


def draw_frontier_target(target):
    if target:
        pygame.draw.circle(screen, YELLOW, (int(target[0]), int(target[1])), 10)


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
    frontiers = []
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
            if slam_map[int(scan_y), int(scan_x)] == 255:
                frontiers.append((scan_x, scan_y))
            cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, 128, -1)
    return frontiers


def find_frontier_target(frontiers, robot_x, robot_y):
    if frontiers:
        # Prioritize frontiers based on distance from the robot and proximity to boundaries
        frontiers.sort(key=lambda point: (visited_map[int(point[1]), int(point[0])],
                                          -math.hypot(point[0] - robot_x, point[1] - robot_y)))
        return frontiers[0]
    return None

# Main game loop
running = True
frontier_target = None
while running:
    screen.fill(WHITE)
    frame_count += 1
    random_movement_counter += 1

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    # Perform lidar scan and get frontiers
    frontiers = lidar_scan(robot_x, robot_y, robot_angle)

    # If there's no current target or the target is out of bounds, find a new one
    if (not frontier_target or frontier_target[0] < 0 or frontier_target[0] >= map_size[1] or
            frontier_target[1] < 0 or frontier_target[1] >= map_size[0]):
        frontier_target = find_frontier_target(frontiers, robot_x, robot_y)

    # Move towards the current frontier target
    if frontier_target:
        target_x, target_y = frontier_target
        angle_to_target = math.degrees(math.atan2(target_y - robot_y, target_x - robot_x))
        angle_diff = (angle_to_target - robot_angle + 360) % 360
        if angle_diff > 180:
            angle_diff -= 360
        if abs(angle_diff) > 10:
            robot_angle = (robot_angle + max(-5, min(5, angle_diff))) % 360
            spin_counter += 1
        else:
            new_x = robot_x + max_speed * math.cos(math.radians(robot_angle))
            new_y = robot_y + max_speed * math.sin(math.radians(robot_angle))
            if not check_collision(new_x, new_y):
                robot_x, robot_y = new_x, new_y
                path_history.append((robot_x, robot_y))  # Record the path for backtracking
                backtrack_stack.append((robot_x, robot_y))  # Record the path for backtracking
                spin_counter = 0  # Reset spin counter
            else:
                wall_following_mode = True  # Enter wall-following mode
                paused_frontier = frontier_target  # Pause the current frontier

        # Check if the robot has been spinning for too long
        if spin_counter > spin_timeout:
            robot_angle = (robot_angle + 180) % 360  # Turn around to avoid spinning in place
            spin_counter = 0

    else:
        # Wandering behavior if no frontier is found
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

    # If the robot reaches the target, set frontier_target to None
    if frontier_target:
        distance_to_target = math.hypot(target_x - robot_x, target_y - robot_y)
        if distance_to_target < 10:
            frontier_target = None

    # Update SLAM map with current robot position
    update_slam_map(robot_x, robot_y)

    # Update position history for stuck detection (only after grace period)
    if frame_count > grace_period:
        position_history.append((robot_x, robot_y))
        if len(position_history) == position_history.maxlen:
            avg_distance = sum(math.hypot(position_history[i][0] - position_history[i - 1][0],
                                          position_history[i][1] - position_history[i - 1][1])
                               for i in range(1, len(position_history))) / (len(position_history) - 1)
            if avg_distance < stuck_threshold:
                # Robot is stuck, enter wall-following mode
                wall_following_mode = True
                paused_frontier = frontier_target  # Pause the current frontier target

    # Draw walls, robot, and frontier target
    draw_walls()
    draw_robot(robot_x, robot_y, robot_angle)
    draw_frontier_target(frontier_target)

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

