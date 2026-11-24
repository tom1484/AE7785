import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'team85_final'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/config/cartographer', glob('config/cartographer/*.lua')),
        ('share/' + package_name + '/rviz', glob('rviz/*.rviz')),
        ('share/' + package_name + '/maps', glob('maps/*.yaml') + glob('maps/*.pgm')),
        ('share/' + package_name + '/models', glob('models/*.onnx')),
        ('share/' + package_name + '/sim', glob('sim/*.world')),
    ] + [
        # Gazebo sign models, directory tree kept (sim/models/<Name>/{model.*,meshes/*})
        (os.path.join('share', package_name, d), [os.path.join(d, f) for f in fs])
        for d, _, fs in os.walk('sim/models') if fs
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tom1484',
    maintainer_email='tomchen2003611@gmail.com',
    description='AE7785 final: follow signs through a maze',
    license='Apache-2.0',
    extras_require={'test': ['pytest']},
    entry_points={
        'console_scripts': [
            'sign_detector = team85_final.nodes.sign_detector_node:main',
            'mission = team85_final.nodes.mission_node:main',
            'set_initial_pose = team85_final.nodes.set_initial_pose:main',
            'extract_maze = team85_final.nodes.extract_maze:main',
            'mapping_supervisor = team85_final.nodes.mapping_supervisor:main',
            'drive_cells = team85_final.nodes.drive_cells:main',
        ],
    },
)
