# ROS Talker-Listener 

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


