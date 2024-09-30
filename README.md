# Wall Follower Robot Simulation

This repository hosts the **Wall Follower Robot** simulation. The robot, based on the **e-puck** model, navigates through a maze using proximity sensors to detect walls. The robot explores all possible paths randomly and stops once it reaches the destination. This implementation does not use any specific pathfinding algorithms or PID controllers; instead, it follows a basic logic to avoid obstacles and move along the walls.

## Maze
The robot navigates through a maze as shown below:

[![Maze](https://github.com/Mummanajagadeesh/wall-follower-robot/blob/main/maze_reference.jpg)]

## Demo Video
Click the image below to watch a demo of the simulation in action:

[![YouTube Video](https://img.youtube.com/vi/aZBT_TVdFZY/0.jpg)](https://youtu.be/aZBT_TVdFZY)

## How It Works

### Robot Design
The **e-puck** robot is equipped with multiple proximity sensors positioned around its body. These sensors allow the robot to detect nearby walls and navigate through the maze by adjusting its movement. The robot uses basic sensor readings to make decisions like moving forward or turning to avoid obstacles.

- **Proximity Sensors**: The robot has 8 proximity sensors (`ps0` to `ps7`), which help detect walls or obstacles on all sides of the robot.
- **Motors**: The left and right wheels of the robot are controlled independently, allowing it to move forward, turn in place, or steer based on sensor inputs.

### Maze Exploration Strategy
- The robot starts at a specific initial position in the maze.
- It randomly explores the maze by following walls or avoiding obstacles.
- Once the robot enters the target region, it stops and pauses its motion. There is no attempt to find the shortest path—the robot will explore all possible routes until it reaches the destination.

### Control Logic
- The robot uses the readings from the proximity sensors to detect obstacles (walls) and adjust its speed or direction.
- When it detects a wall directly in front, it turns to the right.
- If there is no wall in front but a wall on the left, the robot moves forward, following the wall.
- If no wall is detected, the robot makes a right turn.
  
The robot continues navigating the maze based on these simple decisions until it reaches the destination.

## Code Explanation

### Controller 1: Robot Position and Wall-Following Logic

```python
from controller import Robot, DistanceSensor
import math

WHEEL_RADIUS = 0.02  
WHEEL_DISTANCE = 0.052 

INITIAL_X = 0.375
INITIAL_Y = -0.875

X_MIN = 18
X_MAX = 20
Y_MIN = -5
Y_MAX = 2

def get_robot_position(left_encoder, right_encoder):
    left_distance = left_encoder.getValue() * WHEEL_RADIUS
    right_distance = right_encoder.getValue() * WHEEL_RADIUS
    distance = (left_distance + right_distance) / 2.0
    theta = (right_distance - left_distance) / WHEEL_DISTANCE
    delta_x = distance * math.cos(theta)
    delta_y = distance * math.sin(theta)
    return delta_x, delta_y

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    left_motor.setPosition(float('inf'))
    right_motor.setPosition(float('inf'))
    
    left_encoder = robot.getDevice('left wheel sensor')
    right_encoder = robot.getDevice('right wheel sensor')
    left_encoder.enable(timestep)
    right_encoder.enable(timestep)
    
    while robot.step(timestep) != -1:
        delta_x, delta_y = get_robot_position(left_encoder, right_encoder)
        current_x = INITIAL_X + delta_x
        current_y = INITIAL_Y + delta_y
        
        print(f"Current Position: ({current_x:.3f}, {current_y:.3f})")
        
        if X_MIN < current_x < X_MAX and Y_MIN < current_y < Y_MAX:
            left_motor.setVelocity(0)
            right_motor.setVelocity(0)
            print("Robot paused in the specified region.")
            continue
        
        prox_sensors = []
        for ind in range(8):
            sensor_name = 'ps' + str(ind)
            prox_sensors.append(robot.getDevice(sensor_name))
            prox_sensors[ind].enable(timestep)
        
        left_wall = prox_sensors[5].getValue() > 80
        front_wall = prox_sensors[7].getValue() > 80
        
        left_speed = max_speed
        right_speed = max_speed
        
        if front_wall:
            print('Turn right in place')
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("Drive forward")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("Turn right")
                left_speed = max_speed / 8
                right_speed = max_speed
                
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)

if __name__ == "__main__":
    my_robot = Robot()
    run_robot(my_robot)
```

**Key Points**:
- **Position Calculation**: The robot's position is tracked based on encoder values, which measure how far the wheels have traveled.
- **Movement Logic**: If there’s a wall in front, the robot turns right in place. If there’s a wall on the left but no wall in front, it drives forward. If no walls are detected, the robot turns right.
- **Stopping Condition**: The robot stops when it reaches a target region within the maze, defined by `X_MIN`, `X_MAX`, `Y_MIN`, and `Y_MAX`.

---

### Controller 2: Basic Wall-Following with Sensors

```python
from controller import Robot

def run_robot(robot):
    timestep = int(robot.getBasicTimeStep())
    max_speed = 6.28
    
    left_motor = robot.getDevice('left wheel motor')
    right_motor = robot.getDevice('right wheel motor')
    
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)
    
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)
    
    prox_sensors = []
    for ind in range(8):
        sensor_name = 'ps' + str(ind)
        prox_sensors.append(robot.getDistanceSensor(sensor_name))
        prox_sensors[ind].enable(timestep)
    
    while robot.step(timestep) != -1:
        for ind in range(8):
            print(prox_sensors[ind].getValue())
        
        left_wall = prox_sensors[5].getValue() > 80
        front_wall = prox_sensors[7].getValue() > 80
        left_corner = prox_sensors[6].getValue() > 80
        
        left_speed = max_speed
        right_speed = max_speed
        
        if front_wall:
            print('Turn right in place')
            left_speed = max_speed
            right_speed = -max_speed
        else:
            if left_wall:
                print("Drive forward")
                left_speed = max_speed
                right_speed = max_speed
            else:
                print("Turn right")
                left_speed = max_speed / 8
                right_speed = max_speed 
            
            if left_corner:
                print("Came too close, drive right")
                left_speed = max_speed
                right_speed = max_speed / 8
                
        left_motor.setVelocity(left_speed)
        right_motor.setVelocity(right_speed)
    
if _name_ == "_main_":
    my_robot = Robot()
    run_robot(my_robot)
```

**Key Points**:
- **Wall-Following Logic**: Similar to the first controller, the robot turns right if it detects a wall in front, moves forward if a left wall is detected, and makes a right turn if no walls are detected.
- **Additional Condition**: If the left corner sensor detects an obstacle, the robot adjusts by turning slightly to the right.

---

## Installation and Usage

### Requirements
- **Webots**: Install the Webots robotics simulator from [here](https://cyberbotics.com/).
- **Python**: Ensure that you have Python installed to run the robot controller.

### Steps to Run
1. Clone this repository to your local machine:
   ```bash
   git clone https://github.com/Mummanajagadeesh/wall-follower-robot-w.git
   cd wall-follower-robot-w
   ```
2. Open Webots and load the **wall_follower_robot.wbt** world file in the simulation folder.
3. Run the simulation and observe the robot navigating through the maze.

---

## Future Enhancements
- **Optimized Pathfinding**: Implementing pathfinding algorithms (e.g., DFS, BFS, A*) to find the shortest path.
- **PID Controller**: Adding a PID controller for smoother wall-following and turning.
- **Maze Complexity**: Introducing more complex mazes with multiple possible solutions and dead-ends.

---

## License
This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.
