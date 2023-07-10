#!/usr/bin/env python3
import rclpy
import time
from rclpy.node import Node
from geometry_msgs.msg import Wrench

class SpoofWrenchDataLoggerNode(Node):

    msg = Wrench()
    fx1 = msg.force.x
    fy1 = msg.force.y
    fz1 = msg.force.z
    tx1 = msg.torque.x
    ty1 = msg.torque.y
    tz1 = msg.torque.z

    afx1 = 0.5
    afy = 1.0
    afz = 1.0
    atx = 1.0
    aty = 1.0
    atz = 1.0

    def __init__(self):
        super().__init__("spoof_wrench_data_logger")
        self.get_logger().info("Spoof wrench data logger started.")
        self.spoof_wrench_data_logger_ = self.create_subscription(Wrench, "/spoofer/wrench", self.data_callback, 10)
        self.spoof_wrench_data_logger_pub = self.create_publisher(Wrench, "/spoofer/wrench_filtered", 10)

    def data_callback(self, data: Wrench):
        t = time.time()

        fx2 = data.force.x
        fy2 = data.force.y
        fz2 = data.force.z
        tx2 = data.torque.x
        ty2 = data.torque.y
        tz2 = data.torque.z
        fdata = Wrench()

        self.afx = 0.25
        self.afy = 0.25
        self.afz = 0.3
        self.atx = 0.5
        self.aty = 0.5
        self.atz = 0.6

        fdata.force.x = self.afx*fx2 + ((1-self.afx)*self.fx1)
        self.fx1 = fdata.force.x
        fdata.force.y = self.afy*fy2 + ((1-self.afy)*self.fy1)
        self.fy1 = fdata.force.y
        fdata.force.z = self.afz*fz2 + ((1-self.afz)*self.fz1)
        self.fz1 = fdata.force.z

        fdata.torque.x = self.atx*tx2 + ((1-self.atx)*self.tx1)
        self.tx1 = fdata.torque.x
        fdata.torque.y = self.aty*ty2 + ((1-self.aty)*self.ty1)
        self.ty1 = fdata.torque.y         
        fdata.torque.z = self.atz*tz2 + ((1-self.atz)*self.tz1)
        self.tz1 = fdata.torque.z
        
        

        self.spoof_wrench_data_logger_pub.publish(fdata)
        #logger_string_force = "Force x: %08.3f  Force y: %08.3f  Force z: %08.3f" % (data.force.x,data.force.y,data.force.z)
        #logger_string_torque = "Torque x: %08.3f  Torque y: %08.3f  Torque z: %08.3f" % (data.torque.x,data.torque.y,data.torque.z)
        #self.get_logger().info(logger_string_force + "  " + logger_string_torque)


def main(args=None):
    rclpy.init(args=args)
    node = SpoofWrenchDataLoggerNode()
    rclpy.spin(node)
    rclpy.shutdown()
