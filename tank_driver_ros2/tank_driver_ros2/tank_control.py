#!/usr/bin/env python
# import RPi.GPIO as GPIO

import rclpy
import threading
from rclpy.node import Node

from geometry_msgs.msg import Twist

class TankControlNode(Node):

    def __init__(self):
        super().__init__('tank_driver')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('left_motor_forward', None),
                ('left_motor_backward', None),
                ('right_motor_backward', None),
                ('right_motor_forward', None),
                ('right_motor_pwm', None),
                ('left_motor_pwm', None),
                ('timeout', "2"),
                ('rate', "50"),
                ('max_speed', "0.1"),
                ('wheel_base', "0.15")
            ])

        # self.declare_parameter('left_motor_forward')
        # self.declare_parameter('left_motor_backward')
        # self.declare_parameter('right_motor_forward')
        # self.declare_parameter('right_motor_backward')
        # self.declare_parameter('left_motor_pwm')
        # self.declare_parameter('right_motor_pwm')

        # Pin numbers are the GPIO# value, not the literal pin number.
        pin_left_forward = self.get_parameter('left_motor_forward').get_parameter_value().integer_value
        pin_left_backward = self.get_parameter('left_motor_backward')

        pin_right_forward = self.get_parameter('right_motor_forward')
        pin_right_backward = self.get_parameter('right_motor_backward')

        pin_left_pwm = self.get_parameter('left_motor_pwm')
        pin_right_pwm = self.get_parameter('right_motor_pwm')

        self._timeout = self.get_parameter('timeout')
        self._rate = self.get_parameter('rate')
        self._max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self._wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self._cmd_vel_topic = '/cmd_vel'

        self._last_received = self._time_to_double(self.get_clock().now().to_msg())
        self.get_logger().info('max_speed: "%f"' %self._max_speed)
        
        # Setup subscriber for velocity twist message
        self.vel_subscriber = self.create_subscription(
            Twist,
            self._cmd_vel_topic, 
            self.__velocity_callback,
            10)
        self.vel_subscriber

        self._left_speed_percent = 0
        self._right_speed_percent = 0
        # self._left_motor = MotorGPIO(pin_left_forward, pin_left_backward, pin_left_pwm)
        # self._right_motor = MotorGPIO(pin_right_forward, pin_right_backward, pin_right_pwm)


    """Handle new velocity command message."""
    def __velocity_callback(self, msg):
        
        self._last_received = self._time_to_double(self.get_clock().now().to_msg())
        self.get_logger().info('_last_received: "%f"' % self._last_received)

        # Extract linear and angular velocities from the message
        linear = msg.linear.x
        angular = msg.angular.z

        # Calculate wheel speeds in m/s
        left_speed = linear - angular* float(self._wheel_base)/2
        right_speed = linear + angular*float(self._wheel_base)/2

        # Ideally we'd now use the desired wheel speeds along
        # with data from wheel speed sensors to come up with the
        # power we need to apply to the wheels, but we don't have
        # wheel speed sensors. Instead, we'll simply convert m/s
        # into percent of maximum wheel speed, which gives us a
        # duty cycle that we can apply to each motor.
        self._left_speed_percent = (
            100 * left_speed/self._max_speed)
        self._right_speed_percent = (
            100 * right_speed/self._max_speed)
        self.get_logger().info('left_speed_percent: "%s"' % self._left_speed_percent)

    
    def shutdown(self):
        # Reset pin state.
        print("stop")
        # self._left_motor.stop_motor()
        # self._right_motor.stop_motor()

    def _time_to_double(self, timestemp):
        time_double = timestemp.sec + timestemp.nanosec*1e-9
        return time_double

    def control_motor(self):
        print("move motor")
        self.get_logger().info('_last_received: "%f"' %self._last_received)
        delay = self._time_to_double(self.get_clock().now().to_msg()) - self._last_received
        self.get_logger().info('delay: "%s"' % delay)
        # if delay < self._timeout:
        #     print("move motor")
        # #     self._left_motor.move(self._left_speed_percent)
        # #     self._right_motor.move(self._right_speed_percent)
        # else:
        #     print("stop motor")
        #     self._left_motor.move(0)
        #     self._right_motor.move(0)