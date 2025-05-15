## Repository Overview

This repository contains the implementation and explanation of the control codes used with Turtlesim in ROS. It also includes the virtual metrics evaluated by each member of the team to analyze and validate the behavior of the robot in simulation.

Alejandra:
## Virtual Metric in Turtlesim for Motion Validation

To validate the robot's motion control strategy, a virtual metric was implemented in the Turtlesim environment using ROS. The objective was to simulate a linear motion along the `x`-axis from `x = 0` to `x = 7`, mimicking the physical robot's task of moving forward by 10 cm. In both cases, a proportional (P) controller was used to adjust the robot's velocity based on the remaining distance to the goal.

This virtual metric was crucial for two main reasons:

1. **Controlled Testing**: It allowed for safe and controlled testing of the control logic in a simulated environment without the uncertainties present in real hardware, such as surface friction or motor inconsistencies.
2. **Quantifiable Behavior**: It provided a clear and quantifiable way to observe how the velocity changes relative to distance, enabling verification of the control behavior.

The simulation confirmed that the proportional controller produced smooth deceleration as the robot approached the target, and that the velocity-error relationship was consistent with theoretical expectations.

By comparing this ideal behavior with the physical robot—whose speed was tested over a 10 cm straight path using PWM-based control—the design team could assess how closely the real system matched the simulated results. Although the real robot lacks feedback mechanisms like encoders, the simulation served as a performance benchmark, highlighting areas where improvements such as closed-loop control could bridge the gap between virtual and physical execution.

The following image shows the graph displaying the error and the turtle's velocity as it moves from x = 0 to x = 7.
![grafica](https://github.com/user-attachments/assets/07c98019-370b-4a56-a812-84e98a2af90a)

And the following image shows the final movement performed by the turtle.
![Imagen de WhatsApp 2025-05-14 a las 16 40 33_0af71a50](https://github.com/user-attachments/assets/0ae05df4-9812-4542-9e95-cb2c03a4ad7c)

In this section, the code used for the Turtlesim simulation will be explained.
This script implements a proportional control strategy in ROS using the Turtlesim simulator. It allows a turtle to move along the x-axis toward a target position (x=7 in this case) provided by the user. The velocity is adjusted proportionally to the remaining distance to the target.

The code:
Script Header
#!/usr/bin/env python3
This line tells the operating system to execute the script using Python 3.

Imports
import rospy #ROS Python client library.
from geometry_msgs.msg import Twist #Message type used to command linear and angular velocity.
from turtlesim.msg import Pose #Message that provides the current position and orientation of the turtle.
from std_msgs.msg import Float32 #Used for publishing simple float values (e.g., error and velocity).

Class: MoveTurtleProportionalControl
__init__ Constructor
rospy.init_node('control_tortuga_x')
Initializes the ROS node with the name control_tortuga_x.

Subscriptions and Publications
self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
self.error_pub = rospy.Publisher('/turtle/error_x', Float32, queue_size=10)
self.vel_pub = rospy.Publisher('/turtle/vel_x', Float32, queue_size=10)

- Subscribes to the turtle's pose.
- Publishes velocity commands.
- Publishes error and velocity values for visualization in tools like PlotJuggler.

Publishing Rate
self.rate = rospy.Rate(10) #Sets a loop rate of 10 Hz (10 iterations per second).

Callback Function
def pose_callback(self, pose):
    self.current_x = pose.x
Updates the current x-position of the turtle each time a new pose is received.

Movement Logic with Proportional Control
def move_turtle_to_desired_x(self, desired_x):
This method moves the turtle toward a desired x-coordinate using proportional control.

Control Loop
Kp = 1 #proportional gain
error_x = desired_x - self.current_x #distance to the target
vel_x = Kp * error_x #computed velocity proportional to the error

Publishing Data
self.error_pub.publish(error_x)
self.vel_pub.publish(vel_x)
Publishes the error and velocity to ROS topics for logging and analysis.

Sending Velocity Commands
twist_msg = Twist()
twist_msg.linear.x = vel_x
self.velocity_publisher.publish(twist_msg)
Commands the turtle to move forward with the calculated velocity.

Stopping Condition
if abs(error_x) < 0.1:
    rospy.loginfo("Desired position reached")
    break
Stops the loop when the turtle is close enough to the target.

User Interaction
def get_desired_x_from_user(self):
    return float(input("Coordenada x: "))
Prompts the user to input a target x-coordinate.
def move_turtle_interactively(self):
    while not rospy.is_shutdown():
        desired_x = self.get_desired_x_from_user()
        self.move_turtle_to_desired_x(desired_x)
Continuously asks for new goals and moves the turtle until the node is shut down.

Main Execution Block
if __name__ == '__main__':
    try:
        move_turtle_proportional = MoveTurtleProportionalControl()
        move_turtle_proportional.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
Creates an instance of the class and starts the interactive movement process. The exception handling ensures graceful shutdown on interruption (e.g., Ctrl+C).
