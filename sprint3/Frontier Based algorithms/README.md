### **Frontier-Based Exploration: Greedy vs. Non-Greedy**

Frontier-based exploration is a popular strategy for autonomous robots tasked with mapping unknown environments. A "frontier" represents the boundary between explored and unexplored regions, guiding the robot to areas that maximize new knowledge.

This document compares **greedy** and **non-greedy** approaches for frontier exploration, explains what frontiers are, and provides a detailed breakdown of the **A*** algorithm with the heuristic used in the non-greedy strategy.

---

### **Frontiers: What Are They?**

A frontier is a location at the boundary between the known (explored) and unknown (unexplored) areas of a robot’s map. In an occupancy grid, this can be detected by identifying cells that:
1. Are marked as **explored** (e.g., value `128`).
2. Have at least one neighbor marked as **unexplored** (e.g., value `255`).

#### **Role of Frontiers**
Frontiers act as targets for the robot to explore further. By navigating to these points, the robot incrementally expands its knowledge of the environment.

---

### **Greedy vs. Non-Greedy Frontier Exploration**

| **Aspect**                 | **Greedy Exploration**                        | **Non-Greedy Exploration**                 |
|----------------------------|-----------------------------------------------|--------------------------------------------|
| **Definition**             | Selects a random single frontier near the robot.   | Considers overall map coverage and efficiency. |
| **Selection Criteria**     | Distance from the robot.                     | Heuristic based on proximity, map value, and coverage potential. |
| **Pathfinding**            | Moves directly to the nearest frontier.      | Uses A* to plan an optimal path to frontiers. |
| **Advantages**             | Simple, fast, and computationally cheap.     | Comprehensive coverage of the entire map.  |
| **Disadvantages**          | Prone to local minima and redundant paths.   | Computationally expensive due to global evaluations. |
| **Best Use Case**          | Small or obstacle-free environments.         | Large, complex environments with many obstacles. |

#### **Greedy Exploration in Action**
- The robot sorts frontiers by their distance from its current position and chooses the closest one.
- Movement is immediate, with no regard for long-term efficiency.
- Example: If there’s a nearby unexplored corner, the robot will head straight for it, even if doing so causes it to miss more significant unexplored areas.

#### **Non-Greedy Exploration in Action**
- The robot evaluates frontiers based on:
  - Their distance from the robot.
  - The value of their position (e.g., unexplored area density).
  - Their strategic importance, such as proximity to walls or potential doorways.
- It uses **A*** pathfinding to optimize its route to the selected frontier.

---

### **Non-Greedy Exploration: A* Pathfinding**

A* (A-star) is a graph-based pathfinding algorithm that finds the shortest path between two points. It achieves this by evaluating each node using the following formula:
```
f(n) = g(n) + h(n)
```

- `f(n)`: Total cost to reach the goal via node `n`.
- `g(n)`: Cost-so-far to reach node `n` from the starting point.
- `h(n)`: Heuristic estimate of the cost to reach the goal from node `n`.

#### **Heuristic in A*** 
The heuristic guides the robot toward the goal efficiently. In this case, the heuristic is the **Manhattan Distance**, calculated as:
```
h(n) = |current_x - goal_x| + |current_y - goal_y|
```

- **Why Manhattan Distance?**
  - It’s simple to compute.
  - It works well for grid-based maps where movement is typically constrained to horizontal and vertical directions.
  - Ensures admissibility, meaning the heuristic never overestimates the true cost, guaranteeing an optimal path.

#### **How A* Works in the Code**
1. **Initialization**:
   - The start node (robot’s position) is added to a priority queue with a cost of `f(start) = 0 + h(start)`.

2. **Node Expansion**:
   - The algorithm repeatedly removes the node with the smallest `f(n)` from the queue.
   - For each neighboring node:
     - Calculate `g(n)` as the current cost plus the movement cost (usually 1 for adjacent cells).
     - Calculate `h(n)` using the Manhattan Distance.
     - Update the total cost `f(n)`.

3. **Path Construction**:
   - When the goal node (frontier) is reached, the algorithm backtracks through a "came-from" dictionary to reconstruct the path.

#### **Example in the Code**
```python
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
```

---

### **Why A* with a Heuristic Is Non-Greedy**
The heuristic balances immediate proximity with the overall goal:
- **Proximity**: The robot still prioritizes nearby frontiers by incorporating `g(n)` in the evaluation.
- **Global Optimization**: The heuristic `h(n)` ensures the robot considers how its movement affects the larger map exploration.

---

### **Overview**
- **Greedy exploration** is quick and efficient in simple environments but may fail to cover the map comprehensively.
- **Non-greedy exploration**, using A* and a heuristic, provides a systematic approach to exploring large or complex environments.
- By combining frontiers and A*, the robot efficiently balances local and global goals, ensuring robust and reliable mapping.
