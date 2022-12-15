#!/usr/bin/env python
# -*- coding: utf-8 -*-

# Based on https://github.com/duckietown/Software/blob/master/catkin_ws/src/dagu_car/src/wheels_driver_node.py
__author__ = 'Rodrigo Muñoz'

# Copiado de https://github.com/Duckietown-Chile/Software/blob/master/catkin_ws/src/duckiebot_driver/scripts/wheels_driver_node.py
# Adaptado a ROS2 en 2022-12 por Raimundo Lorca Correa <raimundo.lorca.c@ug.uchile.cl>

import rclpy
import sys
from rclpy.node import Node
from duckietown_msgs.msg import WheelsCmdStamped, BoolStamped
from duckiebot_driver.serial_interface import DuckietownSerial
from duckiebot_driver.message import DuckietownCommand, DuckietownStatus

class WheelsDriverNode(Node):
	def __init__(self):
		self.node_name = 'wheels_driver_node'
		super().__init__(self.node_name)
		self.get_logger().info("[%s] Initializing " %(self.node_name))
		self.estop=False
		# Driver parameters
		self.declare_parameter('~/baudrate', 57600)
		self.declare_parameter('~/port', '/dev/ttyAMA0')
		self.baudrate = self.get_parameter('~/baudrate').value
		self.port = self.get_parameter('~/port').value
		self.get_logger().info("Using port {} at {} baud/s".format(self.port, self.baudrate))
		
		# Setup driver
		self.driver = DuckietownSerial(self.port, self.baudrate)
		self.cmd = DuckietownCommand()
		self.status = DuckietownStatus()
		
		# Add publisher for wheels command wih execution time
		self.msg_wheels_cmd = WheelsCmdStamped()
		self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, "~/wheels_cmd_executed", qos_profile=0)
		
		# Setup subscribers
		self.control_constant = 1.0
		self.sub_topic = self.create_subscription(WheelsCmdStamped, "~/wheels_cmd", self.wheel_command_cb,  qos_profile=0)
		self.sub_e_stop = self.create_subscription(BoolStamped, "~/emergency_stop", self.emergency_stop_cb,  qos_profile=0)
	
	def wheel_command_cb(self, msg):
		if self.estop:
			self.cmd.pwm_ch1 = 0
			self.cmd.pwm_ch2 = 0
			self.driver.send_command(self.cmd)
			return
		# Velocity conversion @TODO
		
		# Command saturation
		msg.vel_left = min(max(msg.vel_left,-1.0),1.0)
		# Use - to change direction of the motor, right motor
		# is reflected with respect to left motor
		msg.vel_right = -min(max(msg.vel_right,-1.0),1.0)
		
		# L9110 use inverse logic
		if msg.vel_left > 0.0:
			self.cmd.pwm_ch1 = 255 - int(254*msg.vel_left)
		else:
			self.cmd.pwm_ch1 = int(255*msg.vel_left)
		
		if msg.vel_right > 0.0:
			self.cmd.pwm_ch2 = 255 - int(254*msg.vel_right)
		else:
			self.cmd.pwm_ch2 = int(255*msg.vel_right)
		self.driver.send_command(self.cmd)
        
		# debug prints
		# print "left  cmd: {}".format(self.cmd.pwm_ch1)
		# print "right cmd: {}".format(self.cmd.pwm_ch2)
		# Put the wheel commands in a message and publish
		self.msg_wheels_cmd.header = msg.header
		# Record the time the command was given to the wheels_driver
		self.msg_wheels_cmd.header.stamp = self.get_clock().now()
		self.msg_wheels_cmd.vel_left = msg.vel_left
		self.msg_wheels_cmd.vel_right = msg.vel_right
		self.pub_wheels_cmd.publish(self.msg_wheels_cmd)
	
	def emergency_stop_cb(self,msg):
		self.estop = not self.estop
		if self.estop:
			self.get_logger().info("[%s] Emergency Stop Activated")
		else:
			self.get_logger().info("[%s] Emergency Stop Released")
	
	def on_shutdown(self):
		self.cmd.pwm_ch1 = 0
		self.cmd.pwm_ch2 = 0
		self.driver.send_command(self.cmd)
		self.get_logger().info("[%s] Shutting down."%(self.node_name))

def main():
    # Initialize rclpy
	rclpy.init(args=sys.argv)
	# Create node
	wheels_driver = WheelsDriverNode()
	# Setup proper shutdown behavior 
	try: # Keep it spinning to keep the node alive
		rclpy.spin(wheels_driver)
	except KeyboardInterrupt:
		wheels_driver.on_shutdown()

if __name__ == '__main__':
    main()
