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

python
Copiar
Editar
msg.linear.x = KP_LINEAR * distancia
This value controls how fast the turtle moves towards the goal. If the distance is small (less than 0.1 units), the movement stops.

2. Calculating Angle to Goal (ATG)
The ATG is the angle that the turtle needs to rotate to align with the line connecting its current position to the target position. This is calculated using the atan2 function, which returns the angle between the positive X-axis and the vector pointing towards the goal:

python
Copiar
Editar
angulo_deseado = math.atan2(error_y, error_x)
Where error_x and error_y are the differences between the current and target coordinates. This formula calculates the angle in radians towards the target.

The angular error (error_theta) is the difference between the desired angle and the turtle's current orientation (pose_actual.theta). This error is normalized to the range [-π, π] to avoid unnecessary rotations of more than 180 degrees:

python
Copiar
Editar
error_theta = angulo_deseado - pose_actual.theta
error_theta = (error_theta + math.pi) % (2 * math.pi) - math.pi
The KP_ANGULAR constant (the proportional gain for angular control) is used to adjust the turtle's angular velocity (msg.angular.z), which controls the rotation of the turtle towards the goal:

python
Copiar
Editar
msg.angular.z = KP_ANGULAR * error_theta
This ensures that the turtle adjusts its rotation based on the difference in angle between its current orientation and the direction towards the target. When the angular error is small (less than 0.05 radians), the rotation stops.

Summary of how velocities are calculated using DTG and ATG:
Linear velocity: Depends on the Distance to Goal (DTG). The farther the turtle is from the target, the faster it moves. The formula is:

Linear velocity
=
𝐾
𝑃
_
𝐿
𝐼
𝑁
𝐸
𝐴
𝑅
×
Distance
Linear velocity=KP_LINEAR×Distance
Angular velocity: Depends on the Angle to Goal (ATG). The larger the angular difference, the faster the turtle rotates to align with the target. The formula is:

Angular velocity
=
𝐾
𝑃
_
𝐴
𝑁
𝐺
𝑈
𝐿
𝐴
𝑅
×
Angular error
Angular velocity=KP_ANGULAR×Angular error
General Behavior of the Code:
When the turtle is far from the target (large DTG), it will move quickly in a straight line.

When the turtle is close to the target (small DTG), the linear velocity decreases until it stops.

When the turtle is not aligned with the target (large ATG), it will rotate until it is correctly oriented.

This proportional control approach ensures that the turtle moves efficiently towards the target while adjusting its orientation as it gets closer.
