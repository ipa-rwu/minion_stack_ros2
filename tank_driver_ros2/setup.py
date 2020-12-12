from setuptools import setup
import os
package_name = 'tank_driver_ros2'
import glob

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
            'tank_contol = tank_driver_ros2.main:main'
        ],
    },
)
