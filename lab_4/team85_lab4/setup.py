from glob import glob

from setuptools import find_packages, setup

package_name = 'team85_lab4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'wayPoints.txt']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom1484',
    maintainer_email='tomchen2003611@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'scan_processor = team85_lab4.scripts.scan_processor:main',
            'obstacle_state = team85_lab4.scripts.obstacle_state:main',
            'goal = team85_lab4.scripts.goal:main',
            'go_to_goal = team85_lab4.scripts.go_to_goal:main',
            'view_scan = team85_lab4.scripts.view_scan:main',
            'goal_monitor = team85_lab4.scripts.goal_monitor:main',
        ],
    },
)
