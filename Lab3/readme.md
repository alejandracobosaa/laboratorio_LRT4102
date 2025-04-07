Proportional Control for Turtlebot Navigation
This script demonstrates the use of proportional control to move a turtle in the turtlesim simulation of ROS. The script moves the turtle towards a desired position in 2D space and then rotates it to a specified angle. The movement is controlled based on two key concepts:

DTG (Distance to Goal): The distance between the turtle's current position and the target position.

ATG (Angle to Goal): The angle that the turtle needs to rotate to align with the target position.

How the Control Works
Calculating Distance to Goal (DTG)

The DTG is simply the distance between the current position of the turtle (pose_actual.x, pose_actual.y) and the desired position (x_obj, y_obj). This distance is calculated using the Pythagorean theorem:

ini
Copiar
Editar
distancia = math.sqrt(error_x**2 + error_y**2)
Where:

error_x = x_obj - pose_actual.x

error_y = y_obj - pose_actual.y

Once the distance is calculated, the KP_LINEAR constant is used to determine the linear velocity of the turtle:

ini
Copiar
Editar
msg.linear.x = KP_LINEAR * distancia
This means the turtle moves faster when it's farther away and slows down as it gets closer to the target. If the distance is small (less than 0.1 units), the turtle stops moving.

Calculating Angle to Goal (ATG)

The ATG is the angle the turtle needs to face to point directly at the target. It is calculated using the atan2 function:

ini
Copiar
Editar
angulo_deseado = math.atan2(error_y, error_x)
The angular error is the difference between the desired angle and the current orientation:

lua
Copiar
Editar
error_theta = angulo_deseado - pose_actual.theta
error_theta = (error_theta + math.pi) % (2 * math.pi) - math.pi
This keeps the error between -π and π to avoid long rotations. The angular velocity is calculated using:

ini
Copiar
Editar
msg.angular.z = KP_ANGULAR * error_theta
When the error is small (less than 0.05 radians), the turtle stops rotating.

Summary
Linear velocity is based on the DTG: Linear velocity = KP_LINEAR × Distance

Angular velocity is based on the ATG: Angular velocity = KP_ANGULAR × Angular error

Behavior
When the turtle is far, it moves fast in a straight line.

As it gets closer, it slows down.

If it's not facing the goal, it rotates until it aligns.

Requirements
ROS (Robot Operating System)

turtlesim package

Python 3.x

How to Run
Make sure ROS and turtlesim are installed.

Launch turtlesim:

roscore
rosrun turtlesim turtlesim_node

In another terminal, run the script:

rosrun your_package_name your_script_name.py

Enter the X and Y coordinates and desired theta (in degrees) when prompted.

Notes
You may need to adjust KP_LINEAR, KP_ANGULAR, and KP_THETA for smoother behavior.

Make sure the coordinates are less than 10 (within turtlesim boundaries).
