from setuptools import find_packages, setup

package_name = 'team85_object_follower'

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
            'find_object = team85_object_follower.scripts.find_object:main',
            'rotate_robot = team85_object_follower.scripts.rotate_robot:main',
        ],
    },
)
