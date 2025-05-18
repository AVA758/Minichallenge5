import os

from glob import glob
from setuptools import find_packages, setup

package_name = 'minichallenge4'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob(os.path.join('urdf', '*.urdf'))),
        (os.path.join('share', package_name, 'meshes'), glob(os.path.join('meshes', '*.stl'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.py'))),
        (os.path.join('share', package_name, 'rviz'), glob(os.path.join('rviz', '*.rviz'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Paolo Reyes',
    maintainer_email='paolo.alfonso.reyes@gmail.com',
    description='TODO: Package description',
    license='MIT',
    entry_points={
        'console_scripts': [
            'puzzlebot_controller = minichallenge4.puzzlebot_controller:main',
            'puzzlebot_sim = minichallenge4.puzzlebot_sim:main',
            'puzzlebot_localization = minichallenge4.puzzlebot_localization:main',
            'puzzlebot_joint_publisher = minichallenge4.puzzlebot_joint_publisher:main',
            #'wall_following = minichallenge4.wall_following:main',
        ],
    },
)
