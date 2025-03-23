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
AREA_WIDTH = 11.0  # Ancho del área de turtlesim
AREA_HEIGHT = 11.0  # Alto del área de turtlesim

# Dimensiones de las figuras (definidas por el programador)
SQUARE_SIZE = 2.0  # Tamaño del cuadrado
PARALLELOGRAM_BASE = 3.0  # Base del romboide
PARALLELOGRAM_HEIGHT = 2.0  # Altura del romboide
PARALLELOGRAM_ANGLE = math.radians(60)  # Ángulo del romboide en radianes

# Control Proporcional (KP)
KP_LINEAR = 1.0  # Ganancia para movimiento lineal
KP_ANGULAR = 1.0  # Ganancia para movimiento angular

# Variable global para almacenar la posición actual de la tortuga
current_pose = Pose()

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

def pose_callback(pose):
    """Callback para actualizar la posición actual de la tortuga."""
    global current_pose
    current_pose = pose

def move_to_position(pub, x, y, theta):
    """Mueve la tortuga a una posición específica usando control proporcional."""
    msg = Twist()
    rate = rospy.Rate(10)  # Frecuencia de control (10 Hz)

    while not rospy.is_shutdown():
        # Calcula el error en posición y orientación
        error_x = x - current_pose.x
        error_y = y - current_pose.y
        error_theta = theta - current_pose.theta

        # Control Proporcional
        msg.linear.x = KP_LINEAR * error_x
        msg.linear.y = KP_LINEAR * error_y
        msg.angular.z = KP_ANGULAR * error_theta

        # Publica el mensaje de control
        pub.publish(msg)

        # Verifica si la tortuga está cerca de la posición deseada
        if abs(error_x) < 0.1 and abs(error_y) < 0.1 and abs(error_theta) < 0.1:
            break

        rate.sleep()

def set_pen(on):
    """Activa o desactiva el lápiz de la tortuga."""
    rospy.wait_for_service('/turtle1/set_pen')
    set_pen = rospy.ServiceProxy('/turtle1/set_pen', SetPen)
    if on:
        set_pen(255, 255, 255, 2, 0)  # Color blanco, ancho 2, dibujar
    else:
        set_pen(0, 0, 0, 0, 1)  # Sin dibujar

def draw_line(pub, start_x, start_y, end_x, end_y):
    """Dibuja una línea desde (start_x, start_y) hasta (end_x, end_y)."""
    # Mueve la tortuga al punto inicial sin dibujar
    set_pen(False)  # Desactiva el dibujo
    move_to_position(pub, start_x, start_y, 0)

    # Activa el dibujo y mueve la tortuga al punto final
    set_pen(True)  # Activa el dibujo
    move_to_position(pub, end_x, end_y, 0)

def draw_square(pub, start_x, start_y):
    """Dibuja un cuadrado desde la posición inicial."""
    # Define las esquinas del cuadrado
    corners = [
        (start_x, start_y),
        (start_x + SQUARE_SIZE, start_y),
        (start_x + SQUARE_SIZE, start_y + SQUARE_SIZE),
        (start_x, start_y + SQUARE_SIZE),
        (start_x, start_y)  # Regresa al inicio para cerrar el cuadrado
    ]

    # Verifica si todas las esquinas están dentro del área de trabajo
    for corner in corners:
        if not (0 <= corner[0] <= AREA_WIDTH and 0 <= corner[1] <= AREA_HEIGHT):
            print("Error: Una esquina del cuadrado está fuera del área de trabajo.")
            return False

    # Dibuja el cuadrado
    for i in range(len(corners) - 1):
        draw_line(pub, corners[i][0], corners[i][1], corners[i + 1][0], corners[i + 1][1])

    return True

def draw_parallelogram(pub, start_x, start_y):
    """Dibuja un romboide desde la posición inicial."""
    # Define las esquinas del romboide
    corners = [
        (start_x, start_y),
        (start_x + PARALLELOGRAM_BASE, start_y),
        (start_x + PARALLELOGRAM_BASE + PARALLELOGRAM_HEIGHT / math.tan(PARALLELOGRAM_ANGLE), start_y + PARALLELOGRAM_HEIGHT),
        (start_x + PARALLELOGRAM_HEIGHT / math.tan(PARALLELOGRAM_ANGLE), start_y + PARALLELOGRAM_HEIGHT),
        (start_x, start_y)  # Regresa al inicio para cerrar el romboide
    ]

    # Verifica si todas las esquinas están dentro del área de trabajo
    for corner in corners:
        if not (0 <= corner[0] <= AREA_WIDTH and 0 <= corner[1] <= AREA_HEIGHT):
            print("Error: Una esquina del romboide está fuera del área de trabajo.")
            return False

    # Dibuja el romboide
    for i in range(len(corners) - 1):
        draw_line(pub, corners[i][0], corners[i][1], corners[i + 1][0], corners[i + 1][1])

    return True

def clear_screen():
    """Borra la pantalla cerrando y abriendo una nueva tortuga."""
    rospy.wait_for_service('kill')
    kill_turtle = rospy.ServiceProxy('kill', Kill)
    kill_turtle('turtle1')

    rospy.wait_for_service('spawn')
    spawn_turtle = rospy.ServiceProxy('spawn', Spawn)
    spawn_turtle(5.5, 5.5, 0, 'turtle1')  # Spawn en el centro

def main():
    rospy.init_node('turtle_draw_shapes', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)

    while not rospy.is_shutdown():
        print("\nMenú Principal:")
        print("1. Dibujar Cuadrado")
        print("2. Dibujar Romboide")
        print("q. Salir")
        choice = get_key()

        if choice == 'q':
            print("Saliendo...")
            break

        # Solicita la posición inicial
        try:
            start_x = float(input("Ingrese la posición inicial X: "))
            start_y = float(input("Ingrese la posición inicial Y: "))
        except ValueError:
            print("Error: Ingrese valores numéricos válidos.")
            continue

        # Verifica si la posición inicial está dentro del área de trabajo
        if not (0 <= start_x <= AREA_WIDTH and 0 <= start_y <= AREA_HEIGHT):
            print("Error: La posición inicial está fuera del área de trabajo.")
            continue

        # Borra la pantalla antes de dibujar una nueva figura
        clear_screen()

        if choice == '1':
            print("Dibujando Cuadrado...")
            if draw_square(pub, start_x, start_y):
                print("Cuadrado dibujado correctamente.")
        elif choice == '2':
            print("Dibujando Romboide...")
            if draw_parallelogram(pub, start_x, start_y):
                print("Romboide dibujado correctamente.")
        else:
            print("Opción no válida. Intente de nuevo.")

        # Espera a que el usuario presione una tecla para regresar al menú
        print("Presione cualquier tecla para regresar al menú principal...")
        get_key()

if __name__ == '__main__':
    main()

    h