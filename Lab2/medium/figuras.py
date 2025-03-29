#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import time

def move_turtle(linear_speed, angular_speed, duration):
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.sleep(1)  # Pequeña pausa para asegurar la conexión con el nodo
    
    msg = Twist()
    msg.linear.x = linear_speed
    msg.angular.z = angular_speed
    
    start_time = rospy.Time.now().to_sec()
    while rospy.Time.now().to_sec() - start_time < duration:
        pub.publish(msg)
        rospy.sleep(0.1)
    
    msg.linear.x = 0.0
    msg.angular.z = 0.0
    pub.publish(msg)
    rospy.sleep(1)  # Espera antes de la siguiente acción

def draw_square():
    for _ in range(4):
        move_turtle(2.0, 0.0, 2)  # Avanzar
        move_turtle(0.0, 1.57, 1)  # Girar 90°

def draw_triangle():
    for _ in range(3):
        move_turtle(2.0, 0.0, 2)  # Avanzar
        move_turtle(0.0, 2.09, 1)  # Girar 120°

def main():
    rospy.init_node('turtle_shapes', anonymous=True)
    print("Dibujando un cuadrado...")
    draw_square()
    rospy.sleep(2)
    print("Dibujando un triángulo equilátero...")
    draw_triangle()
    print("Dibujo completo.")

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
