#!/usr/bin/env python

import rospy  # Importa la biblioteca de ROS para Python
from find_object_2d.msg import ObjectsStamped  # Importa el mensaje para los objetos detectados
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray  # Importa el mensaje para el control de las articulaciones
from numpy import maximum, minimum
import std_msgs


print('\n HOLAAAAA!\n')


# Define la clase ExploreAndFind
class ExploreAndFind:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('explore_and_find', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bool_pub = rospy.Publisher('/object_detected', Bool, queue_size=1)
        self.joint_pub = rospy.Publisher('/joint_trajectory_point', Float64MultiArray, queue_size=1)  # Publicador para los estados de las juntas

        # Suscribe al tema de objetos detectados y define el callback para procesar los datos
        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.object_callback)


        
        self.not_detected = True
        self.publish()

    def publish(self):
        rate = rospy.Rate(10)  # Publicar a 10 Hz
        while not rospy.is_shutdown():
            if self.not_detected:
                self.bool_pub.publish(False)
                rate.sleep()
            else:
                self.bool_pub.publish(True)
                rate.sleep()


    def object_callback(self, data):
        # Procesa los objetos detectados
        print(data)
        if data.objects.data:  # Si hay algún objeto detectado
            print(data.objects)
            rospy.loginfo(f"Objeto detectado: {data.objects.data[0]}")  # Muestra información sobre el objeto detectado
            self.bool_pub.publish(True)
            self.stop_robot()
            self.lift_arm(data.objects.data[0])

    def stop_robot(self):
        # Detiene el robot publicando un mensaje de velocidad con todos los componentes en cero
        stop_msg = Twist()
        stop_msg.linear.x = 0.0
        stop_msg.angular.z = 0.0
        self.cmd_vel_pub.publish(stop_msg)
        rospy.loginfo("Robot detenido.")

    def clean_joint_states(self, data):
        lower_limits = [0, 0, -1.57, -1.57, -1.57, -1.57, -1]
        upper_limits = [0, 0, 1.57, 1.57, 4, 2, 1.57]
        clean_lower = maximum(lower_limits, data)
        clean_upper = minimum(clean_lower, upper_limits)
        return list(clean_upper)

    def lift_arm(self, num):
        
        if int(num) == 1:
            # Levanta el brazo publicando un mensaje de Float64MultiArray
            joint_pos = Float64MultiArray()
            joint_pos.layout.dim.append(std_msgs.msg.MultiArrayDimension())
            joint_pos.layout.dim[0].label = ''
            joint_pos.layout.dim[0].size = 7
            joint_pos.layout.dim[0].stride = 7
            joint_pos.layout.data_offset = 0
            joint_pos.data = self.clean_joint_states([0, 0, 0.5, -1, 3.14, 1.57, 0])

            rospy.loginfo(f"Publicando Float64MultiArray: {joint_pos}")
            self.joint_pub.publish(joint_pos)
            rospy.loginfo("Brazo levantado, se ha encontrado warning")
        
        elif int(num) == 2:
            
            # Levanta el brazo publicando un mensaje de Float64MultiArray
            joint_pos = Float64MultiArray()
            joint_pos.layout.dim.append(std_msgs.msg.MultiArrayDimension())
            joint_pos.layout.dim[0].label = ''
            joint_pos.layout.dim[0].size = 7
            joint_pos.layout.dim[0].stride = 7
            joint_pos.layout.data_offset = 0
            joint_pos.data = self.clean_joint_states([0, 0, 0.5, -1, 3.14, 1.57, 6.28])

            rospy.loginfo(f"Publicando Float64MultiArray: {joint_pos}")
            self.joint_pub.publish(joint_pos)
            rospy.loginfo("Brazo levantado, se ha encontrado STOP")


    def joint_callback(self, data):
        print("Msg: {}".format(data.header.seq))
        print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight:{0:.2f}rad\n\n".format(data.position[0], data.position[1]))
        print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2:{0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist:{0:.2f}rd\n\n".format(data.position[2], data.position[3], data.position[4], data.position[5]))
        print("Base Position:\n\tBase:{0:.2f}rad\n".format(data.position[6]))
        print("----------")

    def read_joint_states(self):
        rospy.Subscriber('joint_states', JointState, self.joint_callback)


# Punto de entrada del script
if __name__ == '__main__':
    try:
        # Crea una instancia de la clase ExploreAndFind
        explorer = ExploreAndFind()
        # Mantiene el nodo en ejecución
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  # Maneja la excepción si el nodo es interrumpido

