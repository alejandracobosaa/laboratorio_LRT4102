# Proportional Control for Turtlebot Navigation

This script demonstrates the use of proportional control to move a turtle in the `turtlesim` simulation of ROS. The script moves the turtle towards a desired position in 2D space and then rotates it to a specified angle. The movement is controlled based on two key concepts:

- **DTG (Distance to Goal)**: The distance between the turtle's current position and the target position.
- **ATG (Angle to Goal)**: The angle that the turtle needs to rotate to align with the target position.

## How the Control Works

### 1. **Calculating Distance to Goal (DTG)**

The **DTG** is simply the distance between the current position of the turtle (`pose_actual.x`, `pose_actual.y`) and the desired position (specified as `x_obj` and `y_obj`). This distance is calculated using the Pythagorean theorem:

```python
distancia = math.sqrt(error_x**2 + error_y**2)

