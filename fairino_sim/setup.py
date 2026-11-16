from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fairino_sim'


def data_tree(directory):
    """Install every file under directory, keeping the folder layout."""
    entries = []
    for root, _, files in os.walk(directory):
        if files:
            entries.append((os.path.join('share', package_name, root),
                            [os.path.join(root, name) for name in files]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.xacro')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ] + data_tree('tools'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='axbis-jhjung',
    maintainer_email='scarlet777735@gmail.com',
    description='Fairino cobot simulation with ROS2 and Gazebo, Moveit2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
