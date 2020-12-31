from setuptools import setup
import os
import glob

package_name = 'tank_driver_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name +'/launch',
            glob.glob(os.path.join('launch', '*.py'))),
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ipa-rwu',
    maintainer_email='ruichao.wu@ipa.fraunhofer.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tank_contol = tank_driver_ros2.motor_node:main',
            'imu_publisher = tank_driver_ros2.imu_node:main'
        ],
    },
)
