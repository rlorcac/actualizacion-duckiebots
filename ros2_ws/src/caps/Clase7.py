#!/usr/bin/env python

import rclpy #importar ros para python
from rclpy import node 
from std_msgs.msg import String, Int32 # importar mensajes de ROS tipo String y tipo Int32
from geometry_msgs.msg import Twist # importar mensajes de ROS tipo geometry / Twist
from sensor_msgs.msg import Image # importar mensajes de ROS tipo Image
import cv2 # importar libreria opencv
from cv_bridge import CvBridge # importar convertidor de formato de imagenes
import numpy as np # importar libreria numpy
import sys 

class Template(object):
	def __init__(self, path):
		super(Template, self).__init__()
		self.path = path
		self.sub = node.create_subscriber(Image,"/duckiebot/camera_node/image/rect", self.procesar_img) #Cambio de la estructura del subscrber
		self.pub = node.create_publisher(Image,"/duckiebot/camera_node/image/patos_cv", queue_size = 10) #Cambio de la estructura del Publisher
		self.detector =  cv2.CascadeClassifier(self.path)
	def publicar(self):
		pass

	def callback(self,msg):
		pass

	def procesar_img(self, img):
		bridge = CvBridge()
		image = bridge.imgmsg_to_cv2(img, "bgr8")
		# Pasar imagen a escala de grises
		img_gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
		# Definir detecciones
		dets = self.detector.detectMultiScale(img_gray, 1.3, 10)
		# Dibujar rectangulos de cada blob
		for det in dets:
			x,y,w,h = det	
			cv2.rectangle(image, (x,y), (x+w,y+h), (255,255,255), 2)
		# Publicar imagen final
		image_out = cv2.cvtColor(image, cv2.COLOR_HSV2BGR)
		msg = bridge.cv2_to_imgmsg(image_out, "bgr8")
		self.pub.publish(msg) #No estoy seguro que asi sea la estructura

def main():
    rclpy.init(args=sys.argv)
    node = rclpy.create_node('clase 7') #creacion y registro del nodo!
	
    obj = Template('/home/duckiebot/duckietown/catkin_ws/src/desafios_2022/src/cascade3_LBP.xml') # Crea un objeto del tipo Template, cuya definicion se encuentra arriba

	#objeto.publicar() #llama al metodo publicar del objeto obj de tipo Template

    rclpy.spin() #funcion de ROS que evita que el programa termine -  se debe usar en  Subscribers


if __name__ =='__main__':
	main()