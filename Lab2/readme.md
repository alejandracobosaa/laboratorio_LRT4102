# ROS Talker-Listener (BASIC)

This repository demonstrates a basic **Publisher-Subscriber** model using **ROS (Robot Operating System)** with two Python scripts: `talker.py` and `listener.py`.

## Talker (Publisher)
- The `talker.py` script creates a ROS node that **publishes messages** to a specific topic.
- It runs in a loop, continuously sending data.
- The message type is predefined (e.g., `std_msgs/String`).

## Listener (Subscriber)
- The `listener.py` script creates a ROS node that **subscribes** to the same topic.
- It waits for messages from the `talker.py` script and processes them when received.
- The callback function in `listener.py` executes every time a new message is published.

## Synchronization and Functioning
- When both scripts are running, ROS ensures that `listener.py` receives messages published by `talker.py` in real time.
- If the **talker** stops publishing, the **listener** will wait passively for new messages.
- If the **listener** starts before the **talker**, it will remain idle until messages start arriving.
- ROS handles message transport efficiently, ensuring that messages are delivered as long as both nodes are active and properly connected.

## Use Case
This setup is commonly used in **robotics** to transmit sensor data, commands, or status updates between different components of a robotic system.

## How to Run
1. Initialize ROS:
   ```sh
   roscore
   ```
2. Run the talker node:
   ```sh
   rosrun <your_package> talker.py
   ```
3. Run the listener node:
   ```sh
   rosrun <your_package> listener.py
   ```
## How it runs
When talker and listener are running at the same time:
![image](https://github.com/user-attachments/assets/2836d853-8fa1-42f3-a87c-adb0ddcb43c0)

If the talker stops, the listener will remain active but will wait passively until new messages start arriving again.
![image](https://github.com/user-attachments/assets/6bc62c64-8df6-475c-aeb4-65e471da6731)



# Controller Performance Comparison (ADVANCED)

Compares the performance of P, PD, and PID controllers based on velocity error tracking using ROS and PlotJuggler.

The system's performance was evaluated using three different control strategies:
- **Proportional (P) Controller**
![image](https://github.com/user-attachments/assets/cb1fccfc-1cc9-4c9f-b31a-1c955e4f5423)

- **Proportional-Derivative (PD) Controller**
![image](https://github.com/user-attachments/assets/259cc3b6-2554-4c63-a142-958535ace3a9)

- **Proportional-Integral-Derivative (PID) Controller**
![image](https://github.com/user-attachments/assets/c1ec216b-41fc-4eab-8f1c-8fbf116d77d2)

## Performance Analysis
### 1. Proportional (P) Controller
- The velocity error (`/turtle/error_x/data`) follows the velocity command (`/turtle/vel_x/data`), but a **steady-state error** is present.
- The response exhibits an **exponential decay** but takes longer to stabilize.
- The lack of an integral term prevents the complete elimination of the steady-state error.

### 2. Proportional-Derivative (PD) Controller
- The error and velocity data are more closely matched compared to the P controller.
- The **derivative action** reduces overshoot and improves the system's response time.
- The system shows fewer oscillations and a **faster settling time**.
- However, a small steady-state error is still present due to the absence of an integral term.

### 3. Proportional-Integral-Derivative (PID) Controller
- The error and velocity curves are almost **identical**, demonstrating the best tracking performance.
- The **integral term** compensates for steady-state error, enhancing accuracy.
- The **derivative term** minimizes overshoot and oscillations, improving stability.
- This controller provides the **most precise and stable response** among the three options.

## Conclusion
| Controller | Steady-State Error | Response Time | Overshoot | Stability |
|------------|--------------------|---------------|-----------|-----------|
| P          | High               | Slow          | Low       | Moderate  |
| PD         | Moderate           | Faster        | Reduced   | Improved  |
| PID        | Minimal            | Fastest       | Minimal   | Best      |

- The **P controller** is simple but has a steady-state error.
- The **PD controller** improves response time and reduces oscillations but does not completely eliminate steady-state error.
- The **PID controller** offers the best performance in terms of accuracy, stability, and response time.

For precise control applications, the **PID controller** is the most effective choice.


