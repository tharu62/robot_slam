from setuptools import find_packages, setup

package_name = 'slam_core'

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
    maintainer='Edirisinghe Deshan',
    maintainer_email='tharinduxdeshan@gmail.com',
    description='Slam core functions for motor sensor, laser sensor and odometry update from robot and velocity input reading from user',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                'pub = slam_core.pub_odom:main',
                'sub = slam_core.sub_vel:main',
        'pub2 = slam_core.pub_lidar:main',
        ],
    },
)

