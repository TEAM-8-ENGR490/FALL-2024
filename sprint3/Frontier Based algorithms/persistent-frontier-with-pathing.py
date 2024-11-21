import pygame
import math
import cv2
import numpy as np
import heapq
import random

# Initialize pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Simulation - Enhanced Exploration")

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (0, 0, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)
YELLOW = (255, 255, 0)
CYAN = (0, 255, 255)

# Robot properties
robot_x, robot_y = 100, 100
robot_angle = 0
rotation_speed = 12  # Rotation speed in degrees per frame
max_speed = 34  # Maximum speed
collision_radius = 25
backoff_distance = 20  # Increased distance to back off when stuck

# Triangle dimensions (robot shape)
triangle_base = 20
triangle_height = 30

# Walls (Irregular Floor Plan Layout with Doorways)
walls = [
    ((50, 50), (750, 50)),
    ((50, 50), (50, 550)),
    ((750, 50), (750, 550)),
    ((50, 550), (750, 550)),
    ((200, 50), (200, 150)),
    ((200, 250), (200, 400)),
    ((200, 400), (400, 400)),
    ((400, 400), (400, 250)),
    ((400, 150), (400, 50)),
    ((400, 150), (600, 150)),
    ((600, 150), (600, 250)),
    ((600, 400), (600, 550)),
    ((600, 400), (750, 400)),
]

# SLAM-related variables
map_size = (750, 850)
slam_map = np.full(map_size, 255, dtype=np.uint8)

# Lidar sensor simulation
lidar_range = 100
lidar_angle_step = 10

# Clock
clock = pygame.time.Clock()
FPS = 60

# Define constants for map regions
UNEXPLORED = 255
EXPLORED = 128
OBSTACLE = 0

# Priority queue for A* pathfinding
class PriorityQueue:
    def __init__(self):
        self.elements = []

    def empty(self):
        return len(self.elements) == 0

    def put(self, item, priority):
        heapq.heappush(self.elements, (priority, item))

    def get(self):
        return heapq.heappop(self.elements)[1]

# A* pathfinding algorithm
def a_star_search(start, goal, slam_map):
    frontier = PriorityQueue()
    frontier.put(start, 0)
    came_from = {start: None}
    cost_so_far = {start: 0}

    while not frontier.empty():
        current = frontier.get()

        if current == goal:
            break

        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            next_node = (current[0] + dx, current[1] + dy)
            if 0 <= next_node[0] < slam_map.shape[1] and 0 <= next_node[1] < slam_map.shape[0]:
                if slam_map[next_node[1], next_node[0]] == OBSTACLE:
                    continue
                new_cost = cost_so_far[current] + 1
                if next_node not in cost_so_far or new_cost < cost_so_far[next_node]:
                    cost_so_far[next_node] = new_cost
                    priority = new_cost + heuristic(goal, next_node)
                    frontier.put(next_node, priority)
                    came_from[next_node] = current

    return came_from, cost_so_far

def heuristic(a, b):
    (x1, y1) = a
    (x2, y2) = b
    return abs(x1 - x2) + abs(y1 - y2)

def reconstruct_path(came_from, start, goal):
    if goal not in came_from:
        # If the goal is not reachable, return an empty path
        return []
    current = goal
    path = []
    while current != start:
        path.append(current)
        current = came_from[current]
    path.reverse()
    return path

def find_closest_frontier(robot_pos, frontiers):
    closest_distance = float('inf')
    closest_frontier = None
    for frontier in frontiers:
        dist = math.sqrt((frontier[0] - robot_pos[0]) ** 2 + (frontier[1] - robot_pos[1]) ** 2)
        if dist < closest_distance:
            closest_distance = dist
            closest_frontier = frontier
    return closest_frontier

def draw_robot(x, y, angle):
    tip = (x + triangle_height * math.cos(math.radians(angle)),
           y + triangle_height * math.sin(math.radians(angle)))
    base_left = (x + triangle_base / 2 * math.cos(math.radians(angle + 90)),
                 y + triangle_base / 2 * math.sin(math.radians(angle + 90)))
    base_right = (x + triangle_base / 2 * math.cos(math.radians(angle - 90)),
                  y + triangle_base / 2 * math.sin(math.radians(angle - 90)))
    pygame.draw.polygon(screen, BLUE, [tip, base_left, base_right])
    pygame.draw.circle(screen, RED, (int(x), int(y)), collision_radius, 1)

def draw_walls():
    for wall in walls:
        pygame.draw.line(screen, BLACK, wall[0], wall[1], 10)

def check_collision(x, y):
    for wall in walls:
        wall_start, wall_end = wall
        dist = distance_to_line_segment((x, y), wall_start, wall_end)
        if dist < collision_radius:
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
    cv2.circle(slam_map, (int(x), int(y)), 3, EXPLORED, -1)

def lidar_scan(x, y, angle):
    for lidar_angle in range(0, 360, lidar_angle_step):
        scan_angle = angle + lidar_angle
        for distance in range(0, lidar_range, 5):
            scan_x = x + distance * math.cos(math.radians(scan_angle))
            scan_y = y + distance * math.sin(math.radians(scan_angle))
            if scan_x < 50 or scan_x >= 750 or scan_y < 50 or scan_y >= 550:
                break
            if check_collision(scan_x, scan_y):
                cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, OBSTACLE, -1)
                break
            pygame.draw.circle(screen, GREEN, (int(scan_x), int(scan_y)), 2)
            cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, EXPLORED, -1)

def find_frontiers(slam_map):
    frontiers = []
    rows, cols = slam_map.shape
    for y in range(1, rows - 1):
        for x in range(1, cols - 1):
            if slam_map[y, x] == EXPLORED:
                neighbors = [
                    slam_map[y + dy, x + dx]
                    for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
                ]
                if UNEXPLORED in neighbors:
                    frontiers.append((x, y))
    return frontiers

# Main loop variables
target_frontier = None
path = []
current_path_index = 0
recovery_mode = False
frontiers = []

while True:
    screen.fill(WHITE)
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            cv2.destroyAllWindows()
            exit()

    lidar_scan(robot_x, robot_y, robot_angle)
    update_slam_map(robot_x, robot_y)

    if not recovery_mode:
        if not path:
            # Find all frontiers
            frontiers = find_frontiers(slam_map)
            if frontiers:
                # Find the closest frontier
                target_frontier = find_closest_frontier((robot_x, robot_y), frontiers)
                if target_frontier:
                    came_from, _ = a_star_search((int(robot_x), int(robot_y)), target_frontier, slam_map)
                    path = reconstruct_path(came_from, (int(robot_x), int(robot_y)), target_frontier)
                    current_path_index = 0

        if path:
            # Draw the current path as a red dotted line
            for i in range(len(path) - 1):
                if i % 2 == 0:  # Create a dotted effect
                    pygame.draw.line(screen, RED, path[i], path[i + 1], 1)

            target = path[current_path_index]
            new_x, new_y = target

            # Calculate the angle to the next point
            angle_to_target = math.degrees(math.atan2(new_y - robot_y, new_x - robot_x))
            angle_diff = (angle_to_target - robot_angle + 360) % 360
            if angle_diff > 180:
                angle_diff -= 360

            # Rotate towards the target
            if abs(angle_diff) > rotation_speed:
                robot_angle += rotation_speed if angle_diff > 0 else -rotation_speed
            else:
                robot_angle = angle_to_target

            # Move towards the target if facing it
            if abs(angle_diff) <= rotation_speed:
                if not check_collision(new_x, new_y):
                    robot_x, robot_y = new_x, new_y
                    current_path_index += 1
                    if current_path_index >= len(path):
                        path = []
                else:
                    # Back-off at a random angle and re-plan path if collision detected
                    while True:
                        random_backoff_angle = random.uniform(-90, 90)
                        backoff_angle_rad = math.radians(robot_angle + random_backoff_angle)
                        temp_x = robot_x - backoff_distance * math.cos(backoff_angle_rad)
                        temp_y = robot_y - backoff_distance * math.sin(backoff_angle_rad)
                        if not check_collision(temp_x, temp_y):
                            robot_x, robot_y = temp_x, temp_y
                            break
                    recovery_mode = True
                    path = []

    if recovery_mode:
        # Re-plan to the next closest frontier after collision
        frontiers = find_frontiers(slam_map)
        if frontiers:
            target_frontier = find_closest_frontier((robot_x, robot_y), frontiers)
            if target_frontier:
                came_from, _ = a_star_search((int(robot_x), int(robot_y)), target_frontier, slam_map)
                path = reconstruct_path(came_from, (int(robot_x), int(robot_y)), target_frontier)
                current_path_index = 0
                recovery_mode = False

    # Draw all frontiers to keep them visible
    for fx, fy in frontiers:
        pygame.draw.circle(screen, YELLOW, (fx, fy), 3)

    draw_walls()
    draw_robot(robot_x, robot_y, robot_angle)

    pygame.display.flip()
    cv2.imshow('SLAM Map', slam_map)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

pygame.quit()
cv2.destroyAllWindows()
