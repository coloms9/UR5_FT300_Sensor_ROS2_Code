#!/usr/bin/env python3
import rclpy
import math
import time
import numpy as np
from rclpy.node import Node
from geometry_msgs.msg import Wrench
from std_msgs.msg import Float64

class SpoofWrenchDataNode(Node):

    def __init__(self):
            super().__init__("spoof_wrench_pub")
            self.wrench_pub_ = self.create_publisher(Wrench, "/spoofer/wrench", 10)
            #self.wrench_pub_t_ = self.create_publisher(Float64, "/spoofer/wrench/time", 10)
            self.timer = self.create_timer(0.01, self.send_spoof_data)
            self.get_logger().info("Spoof wrench data program started, good luck!")

    def send_spoof_data(self):
        msg = Wrench()
        tmsg = Float64()
        tmsg.data = time.time()
        t = time.time()
        fsdxy = np.random.normal(0,1.2)
        fsdz = np.random.normal(0,0.5)
        tsdxy = np.random.normal(0,0.02)
        tsdz = np.random.normal(0,0.03)
        msg.force.x = 10*math.sin(math.pi*t) + fsdxy
        msg.force.y = 10*math.sin(math.pi*t) + fsdxy
        msg.force.z = 10*math.sin(math.pi*t) + fsdxy
        msg.torque.x = 10*math.sin(math.pi*t) + tsdxy
        msg.torque.y = 10*math.sin(math.pi*t) + tsdxy
        msg.torque.z = 10*math.sin(math.pi*t) + tsdz
        self.wrench_pub_.publish(msg)
        #self.wrench_pub_t_.publish(tmsg)


def main(args=None):
    rclpy.init(args=args)
    node = SpoofWrenchDataNode()
    rclpy.spin(node)
    rclpy.shutdown()