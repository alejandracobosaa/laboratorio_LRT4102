# ROS Turtle Exercises

This repository contains ROS-based exercises involving turtle simulation. The tasks are divided into two main parts:

1. **Spawning Turtles**: A code to spawn 5 turtles at random positions.
2. **Drawing Shapes**: A program that allows the user to select and draw geometric shapes using a turtle.

---

## 1. Spawning Turtles (30 PTS)

### Description
Generate a ROS node that spawns 5 turtles at random positions within the simulation environment.

### Requirements
- The turtles should appear at random poses (position and orientation).
- The code should be well-commented and easy to understand.

---

## 2. Drawing Shapes (70 PTS)

### Description
Create a ROS-based program that allows the user to select and draw geometric shapes using a turtle. The program should include the following features:

1. **Shape Selection**:
   - The user can choose between two sets of shapes:
     - Diamond and Pentagon
     - Triangle and Isosceles Trapezoid
     - Square and Rhomboid
     - Rectangle and Trapezoid
   - The dimensions of each shape are predefined by the programmer.

2. **Initial Position**:
   - The user defines the initial position of the turtle.
   - The program should check if all corners of the shape are within the workspace. If not, it should notify the user and request a new initial position.

3. **Drawing**:
   - If all corners are within the workspace, the program should draw the selected shape.
   - After drawing, the program should inform the user that the drawing is complete.
   - Upon pressing a key, the program should return to the main menu, clear the drawing, and reset the turtle to the initial position.

4. **Control**:
   - The turtle should be controlled using a Proportional (P) controller.

5. **Exit Option**:
   - The program should include an option to exit.

---

### Deliverables
1. **General Description**: A detailed explanation of the programs.
2. **Commented Code**: Well-documented code for both tasks.
3. **Launch Files**: `.launch` files to execute the nodes.
4. **Repository**: The repository should follow the structure used in previous practices.

---
# ROS Turtle Exercises

This repository contains ROS-based exercises involving turtle simulation. The tasks are divided into two main parts:

1. **Spawning Turtles**: A code to spawn 5 turtles at random positions.
2. **Drawing Shapes**: A program that allows the user to select and draw geometric shapes using a turtle.

---

## Exercise 1: Five Turtles


This exercise consists of a Python script that uses ROS (Robot Operating System) to control a turtle in the `turtlesim` simulator. The program allows the user to move the main turtle in the X and Y directions, rotate it, stop it, and create five additional turtles at random positions within the workspace.

---

## Features

- **Control of the Main Turtle**:  
  The user can move the turtle in the X and Y directions, rotate it clockwise or counterclockwise, and stop its movement using specific keys.

- **Creation of Random Turtles**:  
  The program can create five additional turtles at random positions and orientations within the `turtlesim` workspace.

- **Keyboard Interaction**:  
  The program reads keys pressed by the user without requiring the Enter key, allowing for smoother interaction.

- **Use of ROS Services**:  
  The program uses the `Spawn` service of `turtlesim` to create new turtles at random positions.

- **Message Publishing**:  
  The program publishes `Twist` messages to the `/turtle1/cmd_vel` topic to control the movement of the main turtle.

---

## Controls

| Key | Action                        |
|-----|-------------------------------|
| `x` | Move forward in X.            |
| `y` | Move forward in Y.            |
| `a` | Move backward in X.           |
| `b` | Move backward in Y.           |
| `z` | Rotate clockwise.             |
| `r` | Rotate counterclockwise.      |
| `s` | Stop all movement.            |
| `t` | Create five random turtles.   |
| `q` | Exit the program.             |

---

## Code Structure

### Imports
- ROS modules: `rospy`, `Twist`, `Pose`, and the `Spawn` service.
- Python modules: `sys`, `termios`, `tty`, and `random`.

### Functions
- **`get_key()`**: Reads a key from the keyboard without requiring the Enter key.
- **`spawn_turtle(x, y, theta)`**: Calls the `Spawn` service to create a new turtle at the specified position and orientation.

### Main Function
- Initializes the ROS node and configures the publisher to send `Twist` messages to the `/turtle1/cmd_vel` topic.
- Displays a control menu in the console.
- Enters an infinite loop where it waits for user input and performs the corresponding actions (movement, rotation, stop, create turtles, or exit).

---

## Exercise 2: Rhombus and Pentagon

This exercise consists of a Python script that uses ROS (Robot Operating System) to control a turtle in the `turtlesim` simulator. The program allows the user to draw a rhombus and a pentagon in the `turtlesim` workspace based on the initial position provided by the user. The script uses proportional control to move the turtle precisely and includes an interactive menu for selecting the shape to draw.

---

## Features

- **Interactive Menu**:  
  The program provides a console menu that allows the user to choose between drawing a rhombus, a pentagon, or exiting the program.

- **Proportional Control**:  
  The turtle is moved using proportional control (P) to ensure smooth and precise movement to the desired coordinates.

- **Shape Drawing**:  
  - **Rhombus**: Draws a rhombus of a defined size (`RHOMBUS_SIZE`) starting from the initial position provided by the user.
  - **Pentagon**: Draws a pentagon of a defined size (`PENTAGON_SIZE`) starting from the initial position.

- **Position Validation**:  
  The program checks that the initial position and the corners of the shapes are within the `turtlesim` workspace (10x10 units). If any coordinate is out of range, the program displays an error message and does not proceed with the drawing.

- **Screen Clearing**:  
  Before drawing a new shape, the program clears the screen by deleting the current turtle and creating a new one at the center of the workspace.

- **Pen Control**:  
  The program activates and deactivates the turtle's pen to move it without drawing and then draw the lines of the shapes.

---

## Code Structure

### Imports
- ROS modules: `rospy`, `Twist`, `Pose`, and services like `Spawn`, `Kill`, and `SetPen`.
- Python modules: `sys`, `termios`, `tty`, and `math`.

### Global Variables
- `AREA_WIDTH` and `AREA_HEIGHT`: Define the workspace dimensions (10x10 units).
- `RHOMBUS_SIZE` and `PENTAGON_SIZE`: Define the sizes of the rhombus and pentagon.
- `KP_LINEAR` and `KP_ANGULAR`: Proportional control gains for linear and angular movement.
- `current_pose`: Stores the current position of the turtle.

### Functions
- **`get_key()`**: Reads a key from the keyboard without requiring the Enter key.
- **`pose_callback(pose)`**: Updates the current position of the turtle.
- **`move_to_position(pub, x, y, theta)`**: Moves the turtle to a specific position using proportional control.
- **`set_pen(on)`**: Activates or deactivates the turtle's pen.
- **`draw_line(pub, start_x, start_y, end_x, end_y)`**: Draws a line between two points.
- **`draw_rhombus(pub, start_x, start_y)`**: Draws a rhombus starting from the initial position.
- **`draw_pentagon(pub, start_x, start_y)`**: Draws a pentagon starting from the initial position.
- **`clear_screen()`**: Clears the screen by deleting and recreating the turtle.

### Main Function
- Initializes the ROS node and sets up publishers and subscribers.
- Displays an interactive menu for the user to choose between drawing a rhombus, a pentagon, or exiting the program.
- Handles user input, validates the initial position, and draws the selected shape.

---

### Exercise 2: Square and Rhomboid

This exercise consists of a Python script that uses ROS (Robot Operating System) to control a turtle in the `turtlesim` simulator. The program allows the user to draw a square and a rhomboid in the `turtlesim` workspace based on the initial position provided by the user. The script uses proportional control to move the turtle precisely and includes an interactive menu for selecting the shape to draw.

---

## Features

- **Interactive Menu**:  
  The program provides a console menu that allows the user to choose between drawing a square, a rhomboid, or exiting the program.

- **Proportional Control**:  
  The turtle is moved using proportional control (P) to ensure smooth and precise movement to the desired coordinates.

- **Shape Drawing**:  
  - **Square**: Draws a square of a defined size (`SQUARE_SIZE`) starting from the initial position provided by the user.
  - **Rhomboid**: Draws a rhomboid with a defined base (`PARALLELOGRAM_BASE`), height (`PARALLELOGRAM_HEIGHT`), and angle (`PARALLELOGRAM_ANGLE`) starting from the initial position.

- **Position Validation**:  
  The program checks that the initial position and the corners of the shapes are within the `turtlesim` workspace (11x11 units). If any coordinate is out of range, the program displays an error message and does not proceed with the drawing.

- **Screen Clearing**:  
  Before drawing a new shape, the program clears the screen by deleting the current turtle and creating a new one at the center of the workspace.

- **Pen Control**:  
  The program activates and deactivates the turtle's pen to move it without drawing and then draw the lines of the shapes.

---

## Code Structure

### Imports
- ROS modules: `rospy`, `Twist`, `Pose`, and services like `Spawn`, `Kill`, and `SetPen`.
- Python modules: `sys`, `termios`, `tty`, and `math`.

### Global Variables
- `AREA_WIDTH` and `AREA_HEIGHT`: Define the workspace dimensions (11x11 units).
- `SQUARE_SIZE`: Defines the size of the square.
- `PARALLELOGRAM_BASE`, `PARALLELOGRAM_HEIGHT`, and `PARALLELOGRAM_ANGLE`: Define the dimensions of the rhomboid.
- `KP_LINEAR` and `KP_ANGULAR`: Proportional control gains for linear and angular movement.
- `current_pose`: Stores the current position of the turtle.

### Functions
- **`get_key()`**: Reads a key from the keyboard without requiring the Enter key.
- **`pose_callback(pose)`**: Updates the current position of the turtle.
- **`move_to_position(pub, x, y, theta)`**: Moves the turtle to a specific position using proportional control.
- **`set_pen(on)`**: Activates or deactivates the turtle's pen.
- **`draw_line(pub, start_x, start_y, end_x, end_y)`**: Draws a line between two points.
- **`draw_square(pub, start_x, start_y)`**: Draws a square starting from the initial position.
- **`draw_parallelogram(pub, start_x, start_y)`**: Draws a rhomboid starting from the initial position.
- **`clear_screen()`**: Clears the screen by deleting and recreating the turtle.

### Main Function
- Initializes the ROS node and sets up publishers and subscribers.
- Displays an interactive menu for the user to choose between drawing a square, a rhomboid, or exiting the program.
- Handles user input, validates the initial position, and draws the selected shape.

---

### Exercise 2: Triangle and Isosceles Trapezoid

This exercise consists of a Python script that uses ROS (Robot Operating System) to control a turtle in the `turtlesim` simulator. The program allows the user to draw an equilateral triangle and an isosceles trapezoid in the `turtlesim` workspace based on the initial position provided by the user. The script uses proportional control to move the turtle precisely and includes an interactive menu for selecting the shape to draw.

---

## Features

- **Interactive Menu**:  
  The program provides a console menu that allows the user to choose between drawing an equilateral triangle, an isosceles trapezoid, or exiting the program.

- **Proportional Control**:  
  The turtle is moved using proportional control (P) to ensure smooth and precise movement to the desired coordinates.

- **Shape Drawing**:  
  - **Equilateral Triangle**: Draws an equilateral triangle with a defined base size (`BASE_SIZE`) starting from the initial position provided by the user.
  - **Isosceles Trapezoid**: Draws an isosceles trapezoid with a defined base size (`BASE_SIZE`), height (`TRAPEZOID_HEIGHT`), and a smaller top base starting from the initial position.

- **Position Validation**:  
  The program checks that the initial position and the corners of the shapes are within the `turtlesim` workspace (11x11 units). If any coordinate is out of range, the program displays an error message and does not proceed with the drawing.

- **Screen Clearing**:  
  Before drawing a new shape, the program clears the screen by deleting the current turtle and creating a new one at the center of the workspace.

- **Pen Control**:  
  The program activates and deactivates the turtle's pen to move it without drawing and then draw the lines of the shapes.

---

## Code Structure

### Imports
- ROS modules: `rospy`, `Twist`, `Pose`, and services like `Spawn`, `Kill`, and `SetPen`.
- Python modules: `sys`, `termios`, `tty`, and `math`.

### Global Variables
- `AREA_WIDTH` and `AREA_HEIGHT`: Define the workspace dimensions (11x11 units).
- `BASE_SIZE`: Defines the base size for both the triangle and trapezoid.
- `TRAPEZOID_HEIGHT`: Defines the height of the trapezoid.
- `KP_LINEAR` and `KP_ANGULAR`: Proportional control gains for linear and angular movement.
- `current_pose`: Stores the current position of the turtle.

### Functions
- **`get_key()`**: Reads a key from the keyboard without requiring the Enter key.
- **`pose_callback(pose)`**: Updates the current position of the turtle.
- **`move_to_position(pub, x, y, theta)`**: Moves the turtle to a specific position using proportional control.
- **`set_pen(on)`**: Activates or deactivates the turtle's pen.
- **`draw_line(pub, start_x, start_y, end_x, end_y)`**: Draws a line between two points.
- **`draw_triangle(pub, start_x, start_y)`**: Draws an equilateral triangle starting from the initial position.
- **`draw_trapezoid(pub, start_x, start_y)`**: Draws an isosceles trapezoid starting from the initial position.
- **`clear_screen()`**: Clears the screen by deleting and recreating the turtle.

### Main Function
- Initializes the ROS node and sets up publishers and subscribers.
- Displays an interactive menu for the user to choose between drawing an equilateral triangle, an isosceles trapezoid, or exiting the program.
- Handles user input, validates the initial position, and draws the selected shape.

---

