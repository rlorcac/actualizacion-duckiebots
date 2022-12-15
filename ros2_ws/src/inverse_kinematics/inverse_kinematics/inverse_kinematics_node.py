#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from duckietown_msgs.msg import WheelsCmdStamped, Twist2DStamped
from numpy import *
import rospkg
import yaml
import time
import os.path


# Inverse Kinematics Node
# Author: Robert Katzschmann, Shih-Yuan Liu

# Copiado de https://github.com/Duckietown-Chile/Software/blob/master/catkin_ws/src/dagu_car/script/inverse_kinematics_node.py
# Adaptado a ROS2+rclpy en 2022-12 por Raimundo Lorca Correa <raimundo.lorca.c@ug.uchile.cl>

class InverseKinematicsNode(Node):
    def __init__(self, node_name):
        super().__init__(node_name)
        # Get node name and vehicle name
        self.node_name = node_name

        # Set local variable by reading parameters
        self.declare_parameter("~/gain", 1.0)
        self.declare_parameter("~/trim", 0.0)
        self.declare_parameter("~/baseline", 0.1)
        self.declare_parameter("~/radius", 0.0318)
        self.declare_parameter("~/k", 27.0)
        self.declare_parameter("~/limit", 1.0)
        
        self.limit_max = 1.0
        self.limit_min = 0.0
        
        self.gain = self.get_parameter("~/gain").value
        self.trim = self.get_parameter("~/trim").value
        self.baseline = self.get_parameter("~/baseline").value
        self.radius = self.get_parameter("~/radius").value
        self.k = self.get_parameter("~/k").value
        self.limit = self.get_parameter("~/limit").value

        # Setup the publisher and subscriber
        self.sub_car_cmd = self.create_subscription(Twist2DStamped, "~/car_cmd", self.car_cmd_callback, qos_profile=0)
        self.pub_wheels_cmd = self.create_publisher(WheelsCmdStamped, "~/wheels_cmd", qos_profile=0)
        self.get_logger().info("[%s] Initialized." %(self.node_name))
        self.printValues()

#    def getFilePath(self, name):
#        rospack = rospkg.RosPack()
#        return rospack.get_path('duckietown')+'/config/baseline/calibration/kinematics/' + name + ".yaml"        

    def setLimit(self, value):
        if value > self.limit_max:
            self.get_logger().warn("[%s] limit (%s) larger than max at %s" % (self.node_name, value, self.limit_max))
            limit = self.limit_max
        elif value < self.limit_min:
            self.get_logger().warn("[%s] limit (%s) smaller than allowable min at %s" % (self.node_name, value, self.limit_min))
            limit = self.limit_min
        else:
            limit = value
        return limit

    def printValues(self):
        self.get_logger().info("[%s] gain: %s trim: %s baseline: %s radius: %s k: %s limit: %s" % (self.node_name, self.gain, self.trim, self.baseline, self.radius, self.k, self.limit))

    def car_cmd_callback(self, msg_car_cmd):
        # assuming same motor constants k for both motors
        k_r = self.k
        k_l = self.k

        # adjusting k by gain and trim
        k_r_inv = (self.gain + self.trim) / k_r
        k_l_inv = (self.gain - self.trim) / k_l
        
        omega_r = (msg_car_cmd.v + 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        omega_l = (msg_car_cmd.v - 0.5 * msg_car_cmd.omega * self.baseline) / self.radius
        
        # conversion from motor rotation rate to duty cycle
        # u_r = (gain + trim) (v + 0.5 * omega * b) / (r * k_r)
        u_r = omega_r * k_r_inv
        # u_l = (gain - trim) (v - 0.5 * omega * b) / (r * k_l)
        u_l = omega_l * k_l_inv

        # limiting output to limit, which is 1.0 for the duckiebot
        u_r_limited = max(min(u_r, self.limit), -self.limit)
        u_l_limited = max(min(u_l, self.limit), -self.limit)

        # Put the wheel commands in a message and publish
        msg_wheels_cmd = WheelsCmdStamped()
        msg_wheels_cmd.header.stamp = msg_car_cmd.header.stamp
        msg_wheels_cmd.vel_right = u_r_limited
        msg_wheels_cmd.vel_left = u_l_limited
        self.pub_wheels_cmd.publish(msg_wheels_cmd)

    def setup_parameter(self, param_name, default_value):
        value = rospy.get_param(param_name, default_value)
        # Write to parameter server for transparency
        rospy.set_param(param_name, value)
        self.get_logger().info("[%s] %s = %s " % (self.node_name, param_name, value))
        return value

if __name__ == '__main__':
    rclpy.init()
    inverse_kinematics_node = InverseKinematicsNode('inverse_kinematics_node')
    rclpy.spin(inverse_kinematics_node)
