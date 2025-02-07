from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'nav_farm_walker'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'models/turtlebot_waffle_gps'),
         glob('models/turtlebot_waffle_gps/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='pedro.gonzalez@eia.edu.co',
    description='Demo package for following GPS waypoints with nav2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'logged_waypoint_follower = nav_farm_walker.logged_waypoint_follower:main',
            'interactive_waypoint_follower = nav_farm_walker.interactive_waypoint_follower:main',
            'gps_waypoint_logger = nav_farm_walker.gps_waypoint_logger:main',
            'is_imx_emulator = nav_farm_walker.is_imx_emulator:main',
            'gui = nav_farm_walker.gui:main',
            'sensor_publisher = nav_farm_walker.sensor_publisher:main'
        ],
    },
)
