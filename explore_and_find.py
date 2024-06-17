#!/usr/bin/env python

import rospy  # Importa la biblioteca de ROS para Python
from find_object_2d.msg import ObjectsStamped  # Importa el mensaje para los objetos detectados
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty
from std_msgs.msg import Bool
from std_msgs.msg import Float64MultiArray  # Importa el mensaje para el control de las articulaciones
from numpy import maximum, minimum
import std_msgs
import subprocess


print('\n Començem la exploració!\n')

class ExploreAndFind:
    def __init__(self):
        # Inicializa el nodo ROS
        rospy.init_node('explore_and_find', anonymous=True)
        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)
        self.bool_pub = rospy.Publisher('/object_detected', Bool, queue_size=1)
        self.warn_pub = rospy.Publisher('/warning_detected', Bool, queue_size=1)
        self.joint_pub = rospy.Publisher('/joint_trajectory_point', Float64MultiArray, queue_size=1)  
        # Subsriber per objecte detectat -- Callback
        rospy.Subscriber('/objectsStamped', ObjectsStamped, self.object_callback)
        self.not_detected_stop = True
        self.not_detected_warning = True

        self.publish()
        self.publish_warning()

    def publish(self):
        print(self.not_detected_stop)
        rate = rospy.Rate(10)  # Publicar a 10 Hz
        while not rospy.is_shutdown():
            if self.not_detected_stop:
                self.bool_pub.publish(False)
                rate.sleep()
            else:
                self.bool_pub.publish(True)
                rate.sleep()

    def publish_warning(self):
        rate = rospy.Rate(10)  # Publicar a 10 Hz
        while not rospy.is_shutdown():
            if self.not_detected_warning:
                self.warn_pub.publish(False)
                rate.sleep()


    def object_callback(self, data):
        if data.objects.data:  # Si trobem algun objecte detectat
            #print(data.objects) # Per mirar estructura objecte data
            rospy.loginfo(f"\n\n => Objecte detectat: {data.objects.data[0]}")  # Printem index objecte
            self.bool_pub.publish(True) # Parem el robot
            self.lift_arm(data.objects.data[0])

    
    def clean_joint_states(self, data):
        lower_limits = [0, 0, -1.57, -1.57, -1.57, -1.57, -1]
        upper_limits = [0, 0, 1.57, 1.57, 4, 2, 1.57*3]
        clean_lower = maximum(lower_limits, data)
        clean_upper = minimum(clean_lower, upper_limits)
        return list(clean_upper)

    def lift_arm(self, num):
        
        if int(num) == 1:
            # Aixequem el braç, parem el moviment uns 10 segons, i tornem a posar en marxa la exploració.      
            joint_pos = Float64MultiArray()
            joint_pos.layout.dim.append(std_msgs.msg.MultiArrayDimension())
            joint_pos.layout.dim[0].label = ''
            joint_pos.layout.dim[0].size = 7
            joint_pos.layout.dim[0].stride = 7
            joint_pos.layout.data_offset = 0
            joint_pos.data = self.clean_joint_states([0, 0, 0.5, -1, 3.14, 1.57, 0])

            rospy.loginfo(f"Publicant Float64MultiArray: {joint_pos}")
            self.joint_pub.publish(joint_pos)
            rospy.loginfo("\n\n!!!!!!!!!Braç aixecat, s'ha trobat WARNING!!!!!!!!!!\n\n")
            rospy.sleep(15) # Pausem el robot 15 segons
            self.bool_pub.publish(True)
            self.warn_pub.publish(True) # Avisem pel warning



        elif int(num) == 2:
            
            #self.bool_pub.publish(True) # Parem el robot
            # Girem el el braç 180º i el movem com avís
            joint_pos = Float64MultiArray()
            joint_pos.layout.dim.append(std_msgs.msg.MultiArrayDimension())
            joint_pos.layout.dim[0].label = ''
            joint_pos.layout.dim[0].size = 7
            joint_pos.layout.dim[0].stride = 7
            joint_pos.layout.data_offset = 0
            joint_pos.data = self.clean_joint_states([0, 0, 0.5, -1, 3.14, 1.57, 3.14])

            rospy.loginfo(f"Publicando Float64MultiArray: {joint_pos}")
            self.joint_pub.publish(joint_pos)
            rospy.loginfo("\n\n!!!!!!!!!! Braç aixecat, s'ha trobat STOP, parem el moviment !!!!!!!!!!")


    def joint_callback(self, data):
        print("Msg: {}".format(data.header.seq))
        print("Wheel Positions:\n\tLeft: {0:.2f}rad\n\tRight:{0:.2f}rad\n\n".format(data.position[0], data.position[1]))
        print("Joint Positions:\n\tShoulder1: {0:.2f}rad\n\tShoulder2:{0:.2f}rad\n\tElbow: {0:.2f}rad\n\tWrist:{0:.2f}rd\n\n".format(data.position[2], data.position[3], data.position[4], data.position[5]))
        print("Base Position:\n\tBase:{0:.2f}rad\n".format(data.position[6]))
        print("----------")

    def read_joint_states(self):
        rospy.Subscriber('joint_states', JointState, self.joint_callback)


if __name__ == '__main__':
    try:
        explorer = ExploreAndFind()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass  
