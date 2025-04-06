#!/usr/bin/env python3

import rospy
from turtlesim.srv import Spawn, Kill
from turtlesim.msg import Pose
import sys
import termios
import tty
import math

current_pose = Pose()

def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def pose_callback(pose):
    global current_pose
    current_pose = pose

def teleport_turtle(x, y, theta_rad):
    rospy.wait_for_service('kill')
    kill_turtle = rospy.ServiceProxy('kill', Kill)
    kill_turtle('turtle1')

    rospy.wait_for_service('spawn')
    spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
    spawn_turtle(x, y, theta_rad, 'turtle1')  # Aparece directamente en nueva posición

def main():
    rospy.init_node('turtle_spawn_at_xy', anonymous=True)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            x = float(input("Ingrese la coordenada X: "))
            y = float(input("Ingrese la coordenada Y: "))
            theta_deg = float(input("Ingrese el ángulo theta (en grados): "))
            theta_rad = math.radians(theta_deg)

            if x > 10 or y > 10:
                print("Error: Ingrese valores numéricos válidos menores a 10.")
                continue
        except ValueError:
            print("Error: Ingrese valores numéricos válidos.")
            continue

        teleport_turtle(x, y, theta_rad)
        print("Tortuga spawneada en la nueva posición:)")

if __name__ == '__main__':
    main()
