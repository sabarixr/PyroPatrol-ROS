from setuptools import find_packages, setup

package_name = 'frr_navigation'

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
            'teleop_node = frr_navigation.teleop_node:main',
            'esp32_teleop_node = frr_navigation.esp32_teleop_node:main',
            'autonomous_nav_node = frr_navigation.autonomous_nav_node:main',
            'autonomous_firebot_node = frr_navigation.autonomous_firebot_node:main',
            'aruco_follower_node = frr_navigation.aruco_follower_node:main',
        ],
    },
)
