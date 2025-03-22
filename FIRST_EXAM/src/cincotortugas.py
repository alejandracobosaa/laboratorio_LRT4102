#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn
from turtlesim.msg import Pose
import sys
import termios
import tty
import random

def get_key():
    """Lee una tecla del teclado sin necesidad de presionar Enter."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def spawn_turtle(x, y, theta):
    """Llama al servicio de spawn para crear una nueva tortuga."""
    rospy.wait_for_service('/spawn')
    try:
        spawn_service = rospy.ServiceProxy('/spawn', Spawn)
        spawn_service(x, y, theta, "turtle_" + str(random.randint(1, 1000)))
    except rospy.ServiceException as e:
        rospy.logerr("Error al llamar al servicio de spawn: %s", e)

def main():
    rospy.init_node('turtle_random_spawner', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    
    print("Controles:")
    print("  x/y: Mover en +X/+Y")
    print("  a/b: Mover en -X/-Y")
    print("  z/r: Girar horario/antihorario")
    print("  s: Detener")
    print("  t: Crear 5 tortugas aleatorias")
    print("  q: Salir")

    while not rospy.is_shutdown():
        key = get_key()
        msg = Twist()
        
        if key == 'x':
            msg.linear.x = 2.0  # Avanza en X
        elif key == 'y':
            msg.linear.y = 2.0  # Avanza en Y
        elif key == 'a':
            msg.linear.x = -2.0  # Retrocede en X
        elif key == 'b':
            msg.linear.y = -2.0  # Retrocede en Y
        elif key == 'z':
            msg.angular.z = 1.0  # Gira en sentido horario
        elif key == 'r':
            msg.angular.z = -1.0  # Gira en sentido antihorario
        elif key == 's':
            msg.linear.x = 0.0  # Detiene el movimiento
            msg.linear.y = 0.0
            msg.angular.z = 0.0
        elif key == 't':
            print("Creando 5 tortugas aleatorias...")
            for _ in range(4):
                x = random.uniform(1.0, 10.0)  # Posición X aleatoria
                y = random.uniform(1.0, 10.0)  # Posición Y aleatoria
                theta = random.uniform(0.0, 6.28)  # Orientación aleatoria (0 a 2π)
                spawn_turtle(x, y, theta)
        elif key == 'q':
            print("Saliendo...")
            break
        
        pub.publish(msg)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass