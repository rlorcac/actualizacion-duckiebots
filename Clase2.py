#!/usr/bin/env python

import rclpy #importar ros para python
from rclpy.node import node 
from sensor_msgs.msg import Joy  # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
import sys 

def _linmap(argrange, tgtrange, arg):
	n = tgtrange[0]
	m = float(tgtrange[1]-tgtrange[0])/float(argrange[1]-argrange[0])
	return n + m * (arg-argrange[0])

def linmap(argrange, tgtrange, iterable):
	ret = [_linmap(argrange, tgtrange, arg) for arg in iterable]
	return ret

def allTheThings(B, controls):
	if B:
		controls = [0 for elem in controls]
	return (B, controls)

class Template(object):
    def __init__(self, args):
        super(Template, self).__init__()
        self.args = args
        self.sub = node.create_subscription(Joy,"/duckiebot/joy",self.callback)#Cambio de la estrucutra del subscriber


	#def publicar(self):

    def callback(self,msg):
		#print(msg.axes)
		#print(msg.buttons)
        B = msg.buttons[1]
        right_hor, right_ver = msg.axes[3:5]
        left_hor, left_ver = msg.axes[0:2]
        controls = (left_hor, left_ver, right_hor, right_ver)
        print(allTheThings(B, linmap([-1,1], [0,1], controls)))

def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('clase 2') #creacion y registro del nodo!

    obj = Template('args') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rclpy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()

