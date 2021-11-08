#!/usr/bin/python
import rospy
from geometry_msgs.msg import WrenchStamped
from geometry_msgs.msg import Wrench
from sensor_msgs.msg import Joy
from std_msgs.msg import Header

class Node():
    def __init__(self, name):
        #Inicializamos el nodo con el nombre que aparece en el primer argumento.
        rospy.init_node(name, anonymous = True)
        #Inicializamos los parametros del nodo
        self.initParameters()
        #Creamos los suscriptores del nodo
        self.initSubscribers()
        #Creamos los publicacdores del nodo
        self.initPublishers()
        #Vamos a la funcion principal del nodo, esta funcion se ejecutara en un loop.
        self.main()
        return
        
    def initParameters(self):
        #Aqui inicializaremos todas las variables del nodo
        self.topic_joy = rospy.get_param("joy_topic" , "/joy")
        self.topic_wre = rospy.get_param("wrench_topic" , "/wrench")
        self.frc_scale = rospy.get_param("frc_scale" , 20)
        self.trq_scale = rospy.get_param("trq_scale" , 20)
        self.stamped = rospy.get_param("wrench_stamped", False)
        self.frame_id = rospy.get_param("frame_id", "frc_frame")
        self.msg_joy = Joy()
        self.msg_wre_stamp = WrenchStamped()
        self.msg_wre = Wrench()
        self.change_joy = False
        self.rate = rospy.Rate(rospy.get_param("rate", 30))
        return
        
    def initSubscribers(self):
        #Aqui inicializaremos los suscriptrores
        self.sub_joy = rospy.Subscriber(self.topic_joy, Joy, self.callback_joy)
        return
        
    def initPublishers(self):
        #Aqui inicializaremos los publicadores
        if self.stamped:
            self.pub_wre = rospy.Publisher(self.topic_wre, WrenchStamped, queue_size = 10)
        else:
            self.pub_wre = rospy.Publisher(self.topic_wre, Wrench, queue_size = 10)
        return  
        
    def callback_joy(self, msg):
        #El argumento msg tomara el valor del mensaje recibido
        axes = msg.axes
        button = msg.buttons[5]
        self.frc = self.frc_scale*(axes[1] +  axes[4])/2.0
        self.trq = self.frc_scale*(axes[4] -  axes[1])/2.0
        if button == 1:
            self.change_joy = True
        else:
            self.change_joy = False
        return
        
    def make_msg_wrench(self):
        header = Header()
        header.seq = 0
        header.stamp.secs = rospy.get_rostime().secs
        header.stamp.nsecs = rospy.get_rostime().nsecs
        header.frame_id = self.frame_id
        
        wrench = Wrench()
        wrench.force.x = self.frc
        wrench.force.y = self.frc
        wrench.force.z = self.frc
        wrench.torque.z = self.trq
        wrench.torque.y = self.trq
        wrench.torque.x = self.trq
        if self.stamped:
            self.msg_wre_stamp = WrenchStamped()
            self.msg_wre_stamp.header = header
            self.msg_wre_stamp.wrench = wrench     
            self.pub_wre.publish(self.msg_wre_stamp)
        else:
            self.msg_wre = wrench
            self.pub_wre.publish(self.msg_wre)
        return
    
    def main(self):
        #Aqui desarrollaremos el codigo principal
        print("Nodo OK")
        while not rospy.is_shutdown():
            if self.change_joy:
                self.make_msg_wrench()
                self.change_joy = False
            self.rate.sleep()
                
if __name__=="__main__":
    try:
        print("Iniciando Nodo")
        obj = Node("joy_to_wrench")
    except rospy.ROSInterruptException:
        print("Finalizando Nodo")
