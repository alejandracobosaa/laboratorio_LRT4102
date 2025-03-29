#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
import sys
import termios
import tty

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)  # Lee un solo caracter
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def main():
    rospy.init_node('turtle_keyboard_control', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controla la tortuga con las teclas:")
    print("  f -> Adelante")
    print("  b -> Atrás")
    print("  l -> Izquierda")
    print("  r -> Derecha")
    print("  c -> Girar a la izquierda")
    print("  x -> Girar a la derecha")
    print("  space -> Detenerse")
    print("Presiona 'q' para salir.")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        if key == 'f':
            msg.linear.x = 2.0  # Avanzar
        elif key == 'b':
            msg.linear.x = -2.0  # Retroceder
        elif key == 'l':
            msg.linear.y = 2.0  # Moverse a la izquierda
        elif key == 'r':
            msg.linear.y = -2.0  # Moverse a la derecha
        elif key == 'c':
            msg.angular.z = 1.5  # Girar a la izquierda
        elif key == 'x':
            msg.angular.z = -1.5  # Girar a la derecha
        elif key == ' ':
            msg.linear.x = 0.0
            msg.linear.y = 0.0
            msg.angular.z = 0.0  # Detener el movimiento
        elif key == 'q':  
            print("Saliendo...")
            break  # Sale del loop
        
        pub.publish(msg)

if __name__ == '__main__':
    main()

