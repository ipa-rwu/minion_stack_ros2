#!/usr/bin/env python3
from tank_driver_ros2.tank_control import TankControlNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
import threading


def main():
    rclpy.init()

    tank_control_node = TankControlNode()

    # Spin in a separate thread
    _thread = threading.Thread(target=rclpy.spin, args=(tank_control_node, ), daemon=True)
    _thread.start()  

    rate = tank_control_node.create_rate(tank_control_node.rate)

    try:
        while rclpy.ok():
            tank_control_node.control_motor()
            tank_control_node.pub_imu()
            rate.sleep()
    except KeyboardInterrupt:
        pass
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node obje
    tank_control_node.shutdown()
    tank_control_node.destroy_node()
    rclpy.shutdown()
    _thread.join()