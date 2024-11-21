
### **Random Wandering Algorithms: Comparison and Documentation**

This document explains two random wandering algorithms for robot simulation. Both approaches simulate SLAM-based exploration in environments with obstacles, but they differ in their wandering behavior and collision handling.

---

## **Overview of the Algorithms**

1. **Random Wandering Algorithm (Basic)**:
   - The robot exhibits random wandering behavior with a simple collision-avoidance mechanism.
   - When a collision is detected, the robot changes its direction randomly and continues wandering.

2. **Improved Random Wandering Algorithm**:
   - Builds upon the basic algorithm with enhancements like backtracking, stuck detection, and periodic random movements.
   - Integrates SLAM for more effective exploration and introduces advanced collision-recovery strategies.

---

## **Algorithm Comparison**

| **Aspect**                 | **Basic Algorithm**                               | **Improved Algorithm**                          |
|----------------------------|--------------------------------------------------|------------------------------------------------|
| **Wandering Behavior**     | Simple forward movement with random direction changes on collision. | Periodic random direction changes to avoid stagnation. |
| **Collision Handling**     | Random direction change on collision.            | Backtracking to retrace steps, then randomize direction. |
| **Stuck Detection**        | Not implemented.                                 | Uses position history to detect and recover from being stuck. |
| **Backtracking**           | Not supported.                                   | Retraces steps for a fixed number of frames before resuming wandering. |
| **SLAM Integration**       | Updates map with scanned walls and obstacles.    | Updates map and tracks visited locations to avoid redundant exploration. |
| **Use Case**               | Suitable for small or simple environments.       | Ideal for larger, more complex environments with obstacles. |

---

## **Key Components Explained**

### **1. Wandering Behavior**
- **Basic Algorithm**:
  - Moves forward until a collision is detected.
  - Changes direction randomly by adjusting the robot's angle by ±45 degrees.
  - Ensures the robot does not stay stuck but lacks more advanced recovery mechanisms.

- **Improved Algorithm**:
  - Moves forward with periodic random direction changes, even without collisions, to avoid stagnation.
  - Introduces wall-following mode when stuck to continue exploration in tight spaces.

### **2. Collision Handling**
- **Basic Algorithm**:
  - Upon detecting a collision, the robot:
    - Randomly adjusts its angle (±45 degrees).
    - Immediately resumes forward movement.

- **Improved Algorithm**:
  - If a collision occurs:
    - Enters backtracking mode, retracing its path for a fixed number of steps.
    - After backtracking, randomizes its angle and resumes wandering.

### **3. Stuck Detection (Improved Algorithm Only)**
- Monitors the robot's position history over the last 50 frames.
- If the average movement distance between consecutive positions falls below a threshold, the robot is considered stuck.
- Recovery mechanisms include:
  - Random movement changes.
  - Entering wall-following mode to navigate tight spaces.

### **4. SLAM Integration**
- Both algorithms use LiDAR to map the environment:
  - Casts rays in multiple directions from the robot's position.
  - Updates the SLAM map with:
    - **Obstacles**: Marked as `0`.
    - **Explored areas**: Marked as `128`.

---

## **Code Breakdown**

### **Random Wandering Algorithm (Basic)**

#### **Collision Handling**
```python
if check_collision(new_x, new_y):
    # Change direction randomly on collision
    robot_angle = (robot_angle + random.randint(-45, 45)) % 360
else:
    # Move forward if no collision
    robot_x, robot_y = new_x, new_y
```

#### **Wandering Movement**
- Robot moves in a straight line unless a collision occurs.

---

### **Improved Random Wandering Algorithm**

#### **Stuck Detection**
```python
if len(position_history) == position_history.maxlen:
    avg_distance = sum(
        math.hypot(position_history[i][0] - position_history[i - 1][0],
                   position_history[i][1] - position_history[i - 1][1])
        for i in range(1, len(position_history))
    ) / (len(position_history) - 1)
    if avg_distance < stuck_threshold:
        wall_following_mode = True
```

#### **Backtracking**
- When the robot detects a collision:
  - Enters backtracking mode, retracing its previous steps.
```python
if backtracking:
    new_x = robot_x + backtracking_distance * math.cos(math.radians(robot_angle))
    new_y = robot_y + backtracking_distance * math.sin(math.radians(robot_angle))
    if not check_collision(new_x, new_y):
        robot_x, robot_y = new_x, new_y
    else:
        backtracking = False
```

#### **Random Movement**
- Periodically changes direction to avoid stagnation, even without collisions.
```python
if random_movement_counter > random_movement_interval:
    robot_angle = (robot_angle + random.randint(-45, 45)) % 360
    random_movement_counter = 0
```

---

## **Use Cases and Best Practices**

1. **Basic Algorithm**:
   - Use for testing simple environments or small-scale simulations.
   - Good for quick prototyping and scenarios where advanced recovery is unnecessary.

2. **Improved Algorithm**:
   - Ideal for complex environments with obstacles and narrow passages.
   - Ensures better map coverage and reduces redundant exploration.

---

### **Conclusion**

The **basic random wandering algorithm** is effective for straightforward scenarios, while the **improved version** is designed for robust exploration in complex settings. By adding backtracking, stuck detection, and random periodic movements, the improved algorithm ensures a balance between simplicity and efficiency.
