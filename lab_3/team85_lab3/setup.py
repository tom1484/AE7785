from setuptools import find_packages, setup

package_name = 'team85_lab3'

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
            'detect_object = team85_lab3.scripts.detect_object:main',
            'get_object_range = team85_lab3.scripts.get_object_range:main',
            'chase_object = team85_lab3.scripts.chase_object:main',
        ],
    },
)
