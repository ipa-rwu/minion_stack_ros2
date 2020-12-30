#!/usr/bin/env python3
import RPi.GPIO as GPIO

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from tank_driver_ros2.motor_control import MotorGPIO
from sensor_msgs.msg import Imu
from tank_driver_ros2.MPU6050 import MPU6050
import math
from scipy.spatial.transform import Rotation

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
                ('timeout', '2'),
                ('rate', "50"),
                ('max_speed', "0.1"),
                ('wheel_base', "0.15"),
                ('imu_frame_id', 'imu_link'),
                ('imu_bus', '3'),
                ('accel_offset_x', '0.120'),
                ('accel_offset_y', '-0.538'),
                ('accel_offset_z', '10.155'),
                ('gyro_offset_x', '0.430'),
                ('gyro_offset_y', '0.120'),
                ('gyro_offset_z', '0.690'),
            ])

        # Pin numbers are the GPIO# value, not the literal pin number.
        pin_left_forward = self.get_parameter('left_motor_forward').get_parameter_value().integer_value
        pin_left_backward = self.get_parameter('left_motor_backward').get_parameter_value().integer_value

        pin_right_forward = self.get_parameter('right_motor_forward').get_parameter_value().integer_value
        pin_right_backward = self.get_parameter('right_motor_backward').get_parameter_value().integer_value

        pin_left_pwm = self.get_parameter('left_motor_pwm').get_parameter_value().integer_value
        pin_right_pwm = self.get_parameter('right_motor_pwm').get_parameter_value().integer_value

        self.timeout = self.get_parameter('timeout').get_parameter_value().double_value
        self.rate = self.get_parameter('rate').get_parameter_value().double_value
        self._max_speed = self.get_parameter('max_speed').get_parameter_value().double_value
        self._wheel_base = self.get_parameter('wheel_base').get_parameter_value().double_value

        self._imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value
        self._imu_bus = self.get_parameter('imu_bus').get_parameter_value().integer_value
        _accel_offset_x = self.get_parameter('accel_offset_x').get_parameter_value().double_value
        _accel_offset_y = self.get_parameter('accel_offset_y').get_parameter_value().double_value
        _accel_offset_z = self.get_parameter('accel_offset_z').get_parameter_value().double_value
        _gyro_offset_x = self.get_parameter('gyro_offset_x').get_parameter_value().double_value
        _gyro_offset_y = self.get_parameter('gyro_offset_y').get_parameter_value().double_value
        _gyro_offset_z = self.get_parameter('gyro_offset_z').get_parameter_value().double_value

        self._cmd_vel_topic = '/cmd_vel'
        self._imu_topic = '/imu'

        self._setAccelOffset = [_accel_offset_x, _accel_offset_y, _accel_offset_z]
        self._setGyroOffset = [_gyro_offset_x, _gyro_offset_y, _gyro_offset_z]


        self._last_received = self._time_to_double(self.get_clock().now().to_msg())
        
        # Setup subscriber for velocity twist message
        self.vel_subscriber = self.create_subscription(
            Twist,
            self._cmd_vel_topic, 
            self._velocity_callback,
            10)
        self.vel_subscriber

        # Setup publisher for imu message
        self.publisher_imu = self.create_publisher(Imu, self._imu_topic, 10)
        self._last_pub_imu = self._time_to_double(self.get_clock().now().to_msg())

        self._left_speed_percent = 0
        self._right_speed_percent = 0
        self._left_motor = MotorGPIO(pin_left_forward, pin_left_backward, pin_left_pwm)
        self._right_motor = MotorGPIO(pin_right_forward, pin_right_backward, pin_right_pwm)
        # self.get_logger().info('self.timeout: %f'% self.timeout)

        self._imu = MPU6050(0x68)
        self.imu_data = Imu()
        self.imu_data.header.frame_id = self._imu_frame_id
        # self.imu_data.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        # self.imu_data.angular_velocity_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]
        # self.imu_data.linear_acceleration_covariance = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04]
        self.gyroAngleX = 0
        self.gyroAngleY = 0
        self.yaw = 0


    """Handle new velocity command message."""
    def _velocity_callback(self, msg):
        
        self._last_received = self._time_to_double(self.get_clock().now().to_msg())

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
        # self.get_logger().info('left_speed_percent: "%s"' % self._left_speed_percent)

    def pub_imu(self):
        now = self.get_clock().now().to_msg()
        elapsedTime = self._time_to_double(now) - self._last_pub_imu

        self.imu_data.header.stamp = now
        self.imu_data.linear_acceleration.x = self._imu.get_accel_data()['x'] - self._setAccelOffset[0]
        self.imu_data.linear_acceleration.y = self._imu.get_accel_data()['y'] - self._setAccelOffset[1]
        self.imu_data.linear_acceleration.z = self._imu.get_accel_data()['z'] - self._setAccelOffset[2]

        GyroX = self._imu.get_gyro_data()['x']-self._setGyroOffset[0]
        GyroY = self._imu.get_gyro_data()['y']-self._setGyroOffset[1]
        GyroZ = self._imu.get_gyro_data()['z']-self._setGyroOffset[2]

        self.imu_data.angular_velocity.x = math.radians(GyroX)  
        self.imu_data.angular_velocity.y = math.radians(GyroY)  
        self.imu_data.angular_velocity.z = math.radians(GyroZ)

        accelAngle = self._imu.get_accel_angle(self._setAccelOffset)

        self.gyroAngleX = self.gyroAngleX + GyroX * elapsedTime
        self.gyroAngleY = self.gyroAngleY + GyroY * elapsedTime

        roll = 0.96 * self.gyroAngleX + 0.04 * accelAngle['x']
        pitch = 0.96 * self.gyroAngleY + 0.04 * accelAngle['y']
        yaw =  self.imu_yaw + GyroZ * elapsedTime

        rot = Rotation.from_euler('xyz', [roll, pitch, yaw], degrees=True)
        rot_quat = rot.as_quat()

        self.imu_data.orientation.x = rot_quat[0]
        self.imu_data.orientation.y = rot_quat[1]
        self.imu_data.orientation.z = rot_quat[2]
        self.imu_data.orientation.w = rot_quat[3]

        self._last_pub_imu = self._time_to_double(now)

    def shutdown(self):
        # Reset pin state.
        self._left_motor.stop_motor()
        self._right_motor.stop_motor()

    def _time_to_double(self, timestemp):
        time_double = timestemp.sec + timestemp.nanosec*1e-9
        return time_double

    def control_motor(self):
        delay = self._time_to_double(self.get_clock().now().to_msg()) - self._last_received

        if delay <= self.timeout:
            # self.get_logger().info('move motor')
            self._left_motor.move(self._left_speed_percent)
            self._right_motor.move(self._right_speed_percent)
        elif delay > self.timeout:
            # self.get_logger().info('stop motor')
            self._left_motor.move(0)
            self._right_motor.move(0)