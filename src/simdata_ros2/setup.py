from setuptools import find_packages, setup

package_name = 'simdata_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,

    maintainer='Anjith Bineesh',
    maintainer_email='anjithbineesh@gmail.com',

    description='ROS 2 integration for the SimData virtual robotic sensor library.',

    license='Apache-2.0',

    extras_require={
        'test': [
            'pytest',
        ],
    },

    entry_points={
        'console_scripts': [
            'lidar_test = simdata_ros2.lidar_test:main',
            'imu_test = simdata_ros2.imu_test:main',
            'odom_test = simdata_ros2.odom_test:main',
            'simdata_replay = simdata_ros2.replay_node:main',
        ],
    },
)