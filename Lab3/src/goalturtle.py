#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

# Ganancias del control proporcional
KP_LINEAR = 1.5
KP_ANGULAR = 6.0
KP_THETA = 4.0

# Variables globales
pose_actual = Pose()

def pose_callback(data):
    """Actualiza la posición actual de la tortuga."""
    global pose_actual
    pose_actual = data

def mover_a_objetivo(pub, x_obj, y_obj):
    """Mueve la tortuga hacia la posición deseada usando control proporcional."""
    rate = rospy.Rate(10)
    msg = Twist()

    while not rospy.is_shutdown():
        error_x = x_obj - pose_actual.x
        error_y = y_obj - pose_actual.y

        distancia = math.sqrt(error_x**2 + error_y**2)
        angulo_deseado = math.atan2(error_y, error_x)
        error_theta = angulo_deseado - pose_actual.theta
        error_theta = (error_theta + math.pi) % (2 * math.pi) - math.pi  # Normaliza a [-π, π]

        msg.linear.x = KP_LINEAR * distancia
        msg.angular.z = KP_ANGULAR * error_theta

        pub.publish(msg)

        rospy.loginfo("DTG: %.2f | Error ángulo: %.2f°", distancia, math.degrees(error_theta))

        if distancia < 0.1:
            break

        rate.sleep()

    # Detener al llegar
    msg.linear.x = 0
    msg.angular.z = 0
    pub.publish(msg)
    rospy.loginfo("Objetivo alcanzado.\n")

def rotar_a_theta(pub, theta_deseado):
    """Gira la tortuga hacia un ángulo específico en radianes."""
    rate = rospy.Rate(10)
    msg = Twist()

    while not rospy.is_shutdown():
        error_theta = theta_deseado - pose_actual.theta
        error_theta = (error_theta + math.pi) % (2 * math.pi) - math.pi

        if abs(error_theta) < 0.05:
            break

        msg.angular.z = KP_THETA * error_theta
        pub.publish(msg)

        rospy.loginfo("Error ángulo final: %.2f°", math.degrees(error_theta))
        rate.sleep()

    msg.angular.z = 0
    pub.publish(msg)

def main():
    rospy.init_node('tortuga_control_proporcional', anonymous=True)
    pub = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
    rospy.Subscriber('/turtle1/pose', Pose, pose_callback)
    rospy.sleep(1)

    while not rospy.is_shutdown():
        try:
            print("\n--- Nueva posición objetivo ---")
            x = float(input("Ingrese la coordenada X: "))
            y = float(input("Ingrese la coordenada Y: "))
            theta_deg = float(input("Ingrese el ángulo theta (en grados): "))
            theta_rad = math.radians(theta_deg)

            if x > 10 or y > 10:
                print("Error: Ingrese valores menores a 10.")
                continue

            mover_a_objetivo(pub, x, y)
            rotar_a_theta(pub, theta_rad)

        except ValueError:
            print("Error: Ingrese valores numéricos válidos.")
            continue

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
