#!/usr/bin/env python

import rclpy #importar ros para python
from rclpy.node import node 
from sensor_msgs.msg import Joy  # joystick
from duckietown_msgs.msg import Twist2DStamped #ruedas
import sys 

def _linmap(argrange, tgtrange, arg):
	n = tgtrange[0]
	m = float(tgtrange[1]-tgtrange[0])/float(argrange[1]-argrange[0])
	return n + m * (arg-argrange[0])

def linmap(argrange, tgtrange, iterable):
	ret = [_linmap(argrange, tgtrange, arg) for arg in iterable]
	return ret

def freno(B, controls):
	if B:
		controls = [0 for elem in controls]
	return (B, controls)

class Template(Node):
    def __init__(self, node_name="test", args=None):
        super().__init__(node_name)
        self.args = args
        self.sub = node.create_subscription(Joy, "/duckiebot/joy", self.callback, qos_profile=0) #Cambio de la estrucutra del subscriber
        self.pub = self.create_publisher(Twist2DStamped,"/duckiebot/wheels_driver_node/car_cmd", qos_profile=0) #Cambio de la estuctura del Publisher

    def publicar(self, B, controls):
        msg = Twist2DStamped()
        (B, controls) = freno(B, controls)
        vel = -controls[1]
        msg.v = vel # [-1,1]
        turn = controls[2]
        msg.omega = _linmap([-1,1], [-15, 15], turn) # [-20,20]
        self.pub.publish(msg)

    def callback(self,msg):
        #print(msg.axes)
        #print(msg.buttons)
        B = msg.buttons[1]
        right_hor, right_ver = msg.axes[3:5]
        left_hor, left_ver = msg.axes[0:2]
        controls = (left_hor, left_ver, right_hor, right_ver)
        self.publicar(B, controls)
		

def main():
    rclpy.init(args=sys.argv)
    node = Template(node_name='clase_3') #creacion y registro del nodo!
    # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rclpy.spin(node) #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()
