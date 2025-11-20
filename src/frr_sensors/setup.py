from setuptools import find_packages, setup

package_name = 'frr_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='alibaba',
    maintainer_email='alibaba@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'imu_node = frr_sensors.imu_node:main',
            'mpu6050_node = frr_sensors.mpu6050_node:main',
            'odometry_node = frr_sensors.odometry_node:main',
            'camera_node = frr_sensors.camera_node:main',
            'simple_camera_node = frr_sensors.simple_camera_node:main',
            'lidar_node = frr_sensors.lidar_node:main',
            'lidar_odometry_node = frr_sensors.lidar_odometry_node:main',
            'obstacle_avoidance_node = frr_sensors.obstacle_avoidance_node:main',
        ],
    },
)
