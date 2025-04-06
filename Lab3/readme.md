# Proportional Control for Turtlebot Navigation

This script demonstrates the use of proportional control to move a turtle in the `turtlesim` simulation of ROS. The script moves the turtle towards a desired position in 2D space and then rotates it to a specified angle. The movement is controlled based on two key concepts:

- **DTG (Distance to Goal)**: The distance between the turtle's current position and the target position.
- **ATG (Angle to Goal)**: The angle that the turtle needs to rotate to align with the target position.

## How the Control Works

### 1. **Calculating Distance to Goal (DTG)**

The **DTG** is simply the distance between the current position of the turtle (`pose_actual.x`, `pose_actual.y`) and the desired position (specified as `x_obj` and `y_obj`). This distance is calculated using the Pythagorean theorem:

```python
distancia = math.sqrt(error_x**2 + error_y**2)

Where:

error_x = x_obj - pose_actual.x

error_y = y_obj - pose_actual.y

Once the distance is calculated, the KP_LINEAR constant (a proportional gain) is used to determine the linear velocity of the turtle (msg.linear.x), which is proportional to the distance to the goal. In other words, the turtle will move faster when it's farther from the target and slow down as it approaches the target.
