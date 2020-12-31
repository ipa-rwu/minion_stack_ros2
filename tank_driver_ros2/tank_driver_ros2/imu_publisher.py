#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from tank_driver_ros2.MPU6050 import MPU6050
import math
from scipy.spatial.transform import Rotation

class ImuPublisherNode(Node):

    def __init__(self):
        super().__init__('imu_publisher')

        self.declare_parameters(
            namespace='',
            parameters=[
                ('imu_rate', "10"),
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
        self.rate = self.get_parameter('imu_rate').get_parameter_value().integer_value
        self._imu_frame_id = self.get_parameter('imu_frame_id').get_parameter_value().string_value
        self._imu_bus = self.get_parameter('imu_bus').get_parameter_value().integer_value
        _accel_offset_x = self.get_parameter('accel_offset_x').get_parameter_value().double_value
        _accel_offset_y = self.get_parameter('accel_offset_y').get_parameter_value().double_value
        _accel_offset_z = self.get_parameter('accel_offset_z').get_parameter_value().double_value
        _gyro_offset_x = self.get_parameter('gyro_offset_x').get_parameter_value().double_value
        _gyro_offset_y = self.get_parameter('gyro_offset_y').get_parameter_value().double_value
        _gyro_offset_z = self.get_parameter('gyro_offset_z').get_parameter_value().double_value

        self._imu_topic = '/imu'

        self._setAccelOffset['x'] = _accel_offset_x
        self._setAccelOffset['y'] = _accel_offset_y
        self._setAccelOffset['z'] = _accel_offset_z
        self._setGyroOffset['x'] = _gyro_offset_x
        self._setGyroOffset['y'] = _gyro_offset_y
        self._setGyroOffset['z'] = _gyro_offset_z

        # Setup publisher for imu message
        self.publisher_imu = self.create_publisher(Imu, self._imu_topic, 10)
        self._last_pub_imu = self._time_to_double(self.get_clock().now().to_msg())

        self._imu = MPU6050(0x68)
        self.imu_data = Imu()
        self.imu_data.header.frame_id = self._imu_frame_id
        # self.imu_data.orientation_covariance = [0.0025, 0, 0, 0, 0.0025, 0, 0, 0, 0.0025]
        # self.imu_data.angular_velocity_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02]
        # self.imu_data.linear_acceleration_covariance = [0.04, 0, 0, 0, 0.04, 0, 0, 0, 0.04]
        self.gyroAngleX = 0
        self.gyroAngleY = 0
        self.imu_yaw = 0

    def pub_imu(self):
        now = self.get_clock().now().to_msg()
        elapsedTime = self._time_to_double(now) - self._last_pub_imu
        accel_data = self._imu.get_accel_data()
        gyro_data = self._imu.get_gyro_data()
        self.imu_data.header.stamp = now
        self.imu_data.linear_acceleration.x = accel_data['x'] - self._setAccelOffset['x']
        self.imu_data.linear_acceleration.y = accel_data['y'] - self._setAccelOffset['y']
        self.imu_data.linear_acceleration.z = accel_data['z'] - self._setAccelOffset['z']

        GyroX = gyro_data['x'] - self._setGyroOffset['x']
        GyroY = gyro_data['y'] - self._setGyroOffset['y']
        GyroZ = gyro_data['z'] - self._setGyroOffset['z']

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
        self.publisher_imu.publish(self.imu_data)

        self._last_pub_imu = self._time_to_double(now)

    def shutdown(self):
        # Reset pin state.
        self._left_motor.stop_motor()
        self._right_motor.stop_motor()

    def _time_to_double(self, timestemp):
        time_double = timestemp.sec + timestemp.nanosec*1e-9
        return time_double