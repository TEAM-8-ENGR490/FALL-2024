import pygame
import math
import cv2
import numpy as np

# Initialize pygame
pygame.init()

# Screen dimensions
WIDTH, HEIGHT = 800, 600
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Robot Simulation - Manual Control")

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
rotation_speed = 5
max_speed = 5
acceleration = 0.2
deceleration = 0.1
collision_radius = 30  # Radius of the circular collision boundary

# Triangle dimensions (Smaller robot)
triangle_base = 20  # Robot base width
triangle_height = 30  # Robot height (from base to tip)

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

# Lidar sensor simulation
lidar_range = 100  # Lidar range in pixels
lidar_angle_step = 10  # Angle step for lidar rays

# Clock
clock = pygame.time.Clock()
FPS = 60

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
    """
    Check if the robot collides with a wall based on its circular collision boundary.
    """
    for wall in walls:
        wall_start, wall_end = wall
        dist = distance_to_line_segment((x, y), wall_start, wall_end)
        if dist < collision_radius:
            return True
    return False

def distance_to_line_segment(point, start, end):
    """
    Calculate the minimum distance from a point to a line segment.
    """
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
    """
    Update the SLAM map with the robot's current position.
    """
    cv2.circle(slam_map, (int(x), int(y)), 3, 0, -1)

def lidar_scan(x, y, angle):
    """
    Simulate a lidar scan by casting rays in multiple directions.
    """
    for lidar_angle in range(0, 360, lidar_angle_step):
        scan_angle = angle + lidar_angle
        for distance in range(0, lidar_range, 5):
            scan_x = x + distance * math.cos(math.radians(scan_angle))
            scan_y = y + distance * math.sin(math.radians(scan_angle))
            if scan_x < 50 or scan_x >= 750 or scan_y < 50 or scan_y >= 550:
                break
            if check_collision(scan_x, scan_y):
                # Mark detected wall on SLAM map
                cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, 0, -1)
                break
            # Draw lidar beam in pygame
            pygame.draw.circle(screen, GREEN, (int(scan_x), int(scan_y)), 2)
            # Mark scanned area on SLAM map
            cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, 128, -1)

# Main game loop
running = True
while running:
    screen.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False

    keys = pygame.key.get_pressed()
    if keys[pygame.K_UP]:
        robot_speed += acceleration
    elif keys[pygame.K_DOWN]:
        robot_speed -= acceleration
    else:
        # Gradually reduce speed when no key is pressed
        if robot_speed > 0:
            robot_speed -= deceleration
        elif robot_speed < 0:
            robot_speed += deceleration

    # Clamp the speed to the maximum speed
    robot_speed = max(-max_speed, min(max_speed, robot_speed))

    if keys[pygame.K_LEFT]:
        robot_angle -= rotation_speed
    if keys[pygame.K_RIGHT]:
        robot_angle += rotation_speed

    # Update robot position
    new_robot_x = robot_x + robot_speed * math.cos(math.radians(robot_angle))
    new_robot_y = robot_y + robot_speed * math.sin(math.radians(robot_angle))

    # Check for collisions before updating the position
    if not check_collision(new_robot_x, new_robot_y):
        robot_x, robot_y = new_robot_x, new_robot_y

    # Update SLAM map with current robot position
    update_slam_map(robot_x, robot_y)

    # Perform lidar scan
    lidar_scan(robot_x, robot_y, robot_angle)

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
