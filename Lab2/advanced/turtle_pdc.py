#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from std_msgs.msg import Float32

class MoveTurtlePDControl:
    def __init__(self):
        rospy.init_node('control_tortuga_x')
        
        # Suscribirse al topic de la posición de la tortuga
        self.pose_subscriber = rospy.Subscriber('/turtle1/pose', Pose, self.pose_callback)
        
        # Publicar en el topic de comandos de movimiento de la tortuga
        self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        
        # Publicar el error y la velocidad para monitoreo en PlotJuggler
        self.error_publisher = rospy.Publisher('/turtle/error_x', Float32, queue_size=10)
        self.velocity_publisher_log = rospy.Publisher('/turtle/vel_x', Float32, queue_size=10)
        
        # Tasa de publicación de mensajes (10 Hz)
        self.rate = rospy.Rate(10)
        
        self.current_x = 0
        self.last_error_x = 0

    def pose_callback(self, pose):
        # Función que se ejecuta cada vez que llega una actualización de la posición de la tortuga
        self.current_x = pose.x

    def move_turtle_to_desired_x(self, desired_x):
        # Constantes de proporcionalidad y derivativa del controlador (ajustables)
        Kp = 1
        Kd = 0.1

        while not rospy.is_shutdown():
            # Calcular el error de posición
            error_x = desired_x - self.current_x
            
            # Calcular la velocidad lineal del movimiento
            vel_x = Kp * error_x + Kd * (error_x - self.last_error_x)
            
            # Guardar el error actual para usarlo en la próxima iteración
            self.last_error_x = error_x
            
            # Crear un mensaje de Twist para enviar el comando de movimiento
            twist_msg = Twist()
            twist_msg.linear.x = vel_x
            
            # Publicar el mensaje
            self.velocity_publisher.publish(twist_msg)
            
            # Publicar el error y la velocidad para análisis
            self.error_publisher.publish(error_x)
            self.velocity_publisher_log.publish(vel_x)
            
            # Imprimir la posición actual, el error y la variable vel_x en la terminal
            rospy.loginfo("Posición actual: %f, Error: %f, Velocidad lineal: %f", self.current_x, error_x, vel_x)
            
            # Verificar si se alcanza la posición deseada
            if abs(error_x) < 0.1:
                rospy.loginfo("Posición deseada alcanzada")
                break
            
            # Esperar hasta la siguiente iteración
            self.rate.sleep()

    def get_desired_x_from_user(self):
        print("Ingrese la posición deseada en el eje x:")
        return float(input("Coordenada x: "))

    def move_turtle_interactively(self):
        while not rospy.is_shutdown():
            # Obtener la posición deseada del usuario
            desired_x = self.get_desired_x_from_user()

            # Mover la tortuga a la posición deseada
            self.move_turtle_to_desired_x(desired_x)

if __name__ == '__main__':
    try:
        move_turtle_pd = MoveTurtlePDControl()
        move_turtle_pd.move_turtle_interactively()
    except rospy.ROSInterruptException:
        pass
