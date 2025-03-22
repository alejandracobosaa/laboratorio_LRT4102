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

## Repository Structure
