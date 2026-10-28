from glob import glob
from setuptools import setup

package_name = 'turtle_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Team',
    maintainer_email='team@example.com',
    description='Turtlesim control servers',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'goto_pose_server = turtle_control.goto_pose_server:main',
            'reset_service = turtle_control.reset_service:main',
            'square_server = turtle_control.square_server:main',
        ],
    },
)
