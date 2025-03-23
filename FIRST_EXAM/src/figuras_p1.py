#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.srv import Spawn, Kill, SetPen
from turtlesim.msg import Pose
import sys
import termios
import tty
import math

# Configuración del área de trabajo
AREA_WIDTH = 11.0
AREA_HEIGHT = 11.0

# Dimensiones de las figuras
BASE_SIZE = 2.0          # Tamaño base para las figuras
TRAPEZOID_HEIGHT = 1.5   # Altura del trapecio

# Control Proporcional (KP)
KP_LINEAR = 1.0
KP_ANGULAR = 1.0

# Variable global para la posición actual
current_pose = Pose()

def get_key():
    """Lee una tecla del teclado."""
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        key = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return key

def pose_callback(pose):
    """Actualiza la posición de la tortuga."""
    global current_pose
    current_pose = pose

def move_to_position(pub, x, y, theta):
    """Mueve la tortuga a una posición específica."""
    msg = Twist()
    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        error_x = x - current_pose.x
        error_y = y - current_pose.y
        error_theta = theta - current_pose.theta

        msg.linear.x = KP_LINEAR * error_x
        msg.linear.y = KP_LINEAR * error_y
        msg.angular.z = KP_ANGULAR * error_theta

        pub.publish(msg)

        if abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_theta) < 0.1:
            break

        rate.sleep()

def set_pen(on):
    """Controla el lápiz de la tortuga."""
    rospy.wait_for_service('/turtle1/set_pen')
    set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    if on:
        set_pen(255, 255, 255, 2, 0)   # Lápiz activado (blanco)
    else:
        set_pen(0, 0, 0, 0, 1)         # Lápiz desactivado

def draw_line(pub, start_x, start_y, end_x, end_y):
    """Dibuja una línea entre dos puntos."""
    set_pen(False)
    move_to_position(pub, start_x, start_y, 0)
    set_pen(True)
    move_to_position(pub, end_x, end_y, 0)

def draw_triangle(pub, start_x, start_y):
    """Dibuja un triángulo equilátero."""
    height = BASE_SIZE * math.sqrt(3)/2  # Altura del triángulo
    corners = [
        (start_x, start_y),
        (start_x + BASE_SIZE, start_y),
        (start_x + BASE_SIZE/2, start_y + height),
        (start_x, start_y)  # Cierra la figura
    ]

    # Verificación de límites
    for corner in corners:
        if not (0 <= corner[0] <= AREA_WIDTH and 0 <= corner[1] <= AREA_HEIGHT):
            print("Error: Figura fuera del área!")
            return False

    # Dibujar
    for i in range(len(corners)-1):
        draw_line(pub, corners[i][0], corners[i][1], corners[i+1][0], corners[i+1][1])
    return True

def draw_trapezoid(pub, start_x, start_y):
    """Dibuja un trapecio isósceles."""
    top_base = BASE_SIZE * 0.6  # Base superior más pequeña
    corners = [
        (start_x, start_y),
        (start_x + BASE_SIZE, start_y),
        (start_x + BASE_SIZE - (BASE_SIZE - top_base)/2, start_y + TRAPEZOID_HEIGHT),
        (start_x + (BASE_SIZE - top_base)/2, start_y + TRAPEZOID_HEIGHT),
        (start_x, start_y)  # Cierra la figura
    ]

    # Verificación de límites
    for corner in corners:
        if not (0 <= corner[0] <= AREA_WIDTH and 0 <= corner[1] <= AREA_HEIGHT):
            print("Error: Figura fuera del área!")
            return False

    # Dibujar
    for i in range(len(corners)-1):
        draw_line(pub, corners[i][0], corners[i][1], corners[i+1][0], corners[i+1][1])
    return True

def clear_screen():
    """Borra la pantalla."""
    rospy.wait_for_service('kill')
    kill_turtle = rospy.ServiceProxy('kill', Kill)
    kill_turtle('turtle1')

    rospy.wait_for_service('spawn')
    spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
    spawn_turtle(5.5, 5.5, 0, 'turtle1')

def main():
    rospy.init_node('turtle_drawer', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    while not rospy.is_shutdown():
        print("\nMenu de Figuras:")
        print("1. Triángulo Equilátero")
        print("2. Trapecio Isósceles")
        print("q. Salir")
        choice = get_key()

        if choice == 'q':
            print("Saliendo...")
            break

        try:
            start_x = float(input("Posición inicial X: "))
            start_y = float(input("Posición inicial Y: "))
        except ValueError:
            print("Error: Solo valores numéricos!")
            continue

        if not (0 <= start_x <= AREA_WIDTH and 0 <= start_y <= AREA_HEIGHT):
            print("Error: Posición inválida!")
            continue

        clear_screen()

        if choice == '1':
            print("Dibujando triángulo...")
            if draw_triangle(pub, start_x, start_y):
                print("Triángulo completado!")
        elif choice == '2':
            print("Dibujando trapecio...")
            if draw_trapezoid(pub, start_x, start_y):
                print("Trapecio completado!")
        else:
            print("Opción inválida!")

        print("Presione una tecla para continuar...")
        get_key()

if __name__ == '__main__':
    main()