from setuptools import find_packages, setup

package_name = 'frr_control'

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
            'motor_driver_node = frr_control.motor_controller_node:main',
            'esp32_bridge_node = frr_control.esp32_bridge_node:main',
            'teleop_node = frr_control.teleop_node:main',
            'teleop_node_clean = frr_control.teleop_node_clean:main',
        ],
    },
)
