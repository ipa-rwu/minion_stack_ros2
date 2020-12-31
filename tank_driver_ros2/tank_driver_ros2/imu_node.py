#!/usr/bin/env python3
from tank_driver_ros2.imu_publisher import ImuPublisherNode
import rclpy
from rclpy.executors import SingleThreadedExecutor
import threading


def main():
    rclpy.init()

    imu_node = ImuPublisherNode()

    # Spin in a separate thread
    _thread = threading.Thread(target=rclpy.spin, args=(imu_node, ), daemon=True)
    _thread.start()  

    rate = imu_node.create_rate(imu_node.rate)

    try:
        while rclpy.ok():
            imu_node.pub_imu()
            rate.sleep()
    except KeyboardInterrupt:
        pass
        # Destroy the node explicitly
        # (optional - otherwise it will be done automatically
        # when the garbage collector destroys the node obje
    imu_node.destroy_node()
    rclpy.shutdown()
    _thread.join()