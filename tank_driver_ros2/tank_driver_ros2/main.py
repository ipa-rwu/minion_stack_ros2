#!/usr/bin/env python3
from tank_driver_ros2.tank_control import TankControlNode
import rclpy

def main():
    rclpy.init()

    tank_control_node = TankControlNode()
    rate = tank_control_node.create_rate(10)

    try:
        while rclpy.ok():
            tank_control_node.control_motor()
            print('Help me body, you are my only hope')            
            rclpy.spin(tank_control_node)
            rate.sleep()
    except KeyboardInterrupt:
        pass
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node obje
    tank_control_node.shutdown()
    tank_control_node.destroy_node()
    rclpy.shutdown()