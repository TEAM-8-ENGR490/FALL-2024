
### **Robot Simulation: Manual Control**

Using Pygame, this simulation allows a user to control a robot manually in an environment with obstacles. The robot can move forward, backward, and rotate using keyboard inputs. The simulation integrates SLAM (Simultaneous Localization and Mapping) with a LiDAR sensor for obstacle detection and map updating.

---

## **Key Features**

1. **Manual Control**:
   - The robot's movement is controlled via the keyboard:
     - **Arrow Keys**:
       - `UP`: Accelerate forward.
       - `DOWN`: Decelerate or reverse.
       - `LEFT` and `RIGHT`: Rotate the robot.

2. **Collision Detection**:
   - Prevents the robot from moving through walls or obstacles using a circular collision boundary.

3. **SLAM Integration**:
   - LiDAR scans the environment to detect walls and obstacles.
   - The map updates in real-time with the robot's movements.



---

## **Code Walkthrough**

### **1. Manual Control with Keyboard Inputs**

- **Accelerate and Decelerate**:
  - The robot's speed increases when the `UP` arrow key is pressed and decreases with the `DOWN` arrow.
  - Gradual deceleration occurs when no keys are pressed.
```python
if keys[pygame.K_UP]:
    robot_speed += acceleration
elif keys[pygame.K_DOWN]:
    robot_speed -= acceleration
else:
    if robot_speed > 0:
        robot_speed -= deceleration
    elif robot_speed < 0:
        robot_speed += deceleration
```

- **Rotation**:
  - The robot rotates when the `LEFT` or `RIGHT` keys are pressed.
```python
if keys[pygame.K_LEFT]:
    robot_angle -= rotation_speed
if keys[pygame.K_RIGHT]:
    robot_angle += rotation_speed
```

- **Speed Limits**:
  - The robot's speed is clamped to a maximum value to prevent unrealistic movement.
```python
robot_speed = max(-max_speed, min(max_speed, robot_speed))
```

---

### **2. Collision Detection**

- **How It Works**:
  - The robot's position is checked against walls using a circular collision boundary.
  - If the robot's new position intersects a wall, the movement is prevented.

- **Distance to Walls**:
  - The distance is calculated using the shortest distance between the robot's position and wall segments.
```python
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
```

- **Update Position**:
  - If no collision is detected, the robot updates its position.
```python
if not check_collision(new_robot_x, new_robot_y):
    robot_x, robot_y = new_robot_x, new_robot_y
```

---

### **3. SLAM and LiDAR Integration**

- **SLAM Map**:
  - The environment is represented as a 2D grid (occupancy grid map) where:
    - White cells (`255`) are unexplored.
    - Gray cells (`128`) are explored.
    - Black cells (`0`) represent obstacles or walls.

- **LiDAR Scanning**:
  - The robot emits beams (rays) in 360 degrees to detect obstacles.
  - Each ray updates the SLAM map by marking explored and obstacle regions.
```python
for lidar_angle in range(0, 360, lidar_angle_step):
    scan_angle = angle + lidar_angle
    for distance in range(0, lidar_range, 5):
        scan_x = x + distance * math.cos(math.radians(scan_angle))
        scan_y = y + distance * math.sin(math.radians(scan_angle))
        if check_collision(scan_x, scan_y):
            cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, 0, -1)
            break
        cv2.circle(slam_map, (int(scan_x), int(scan_y)), 1, 128, -1)
```

---

### **4. Robot Rendering**

- The robot is represented as a blue isosceles triangle, with a red circular collision boundary.
```python
def draw_robot(x, y, angle):
    tip = (x + triangle_height * math.cos(math.radians(angle)),
           y + triangle_height * math.sin(math.radians(angle)))
    base_left = (x + triangle_base / 2 * math.cos(math.radians(angle + 90)),
                 y + triangle_base / 2 * math.sin(math.radians(angle + 90)))
    base_right = (x + triangle_base / 2 * math.cos(math.radians(angle - 90)),
                  y + triangle_base / 2 * math.sin(math.radians(angle - 90)))
    pygame.draw.polygon(screen, BLUE, [tip, base_left, base_right])
    pygame.draw.circle(screen, RED, (int(x), int(y)), collision_radius, 1)
```

---

## **Simulation Features**

1. **Real-Time Map Updates**:
   - The SLAM map is displayed in real-time using OpenCV.
   - Explored areas and obstacles are dynamically updated based on the robot's position and LiDAR scans.

2. **Smooth Movement**:
   - Gradual acceleration and deceleration provide more realistic robot movement.
   - Prevents abrupt changes in speed or direction.

3. **Collision Avoidance**:
   - Ensures the robot does not overlap with walls or obstacles, enhancing realism and safety.
