from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'fairino_workcell'


def data_tree(directory):
    """Install every file under directory, keeping the folder layout."""
    entries = []
    for root, _, files in os.walk(directory):
        if files and '__pycache__' not in root:
            entries.append((os.path.join('share', package_name, root),
                            [os.path.join(root, name) for name in files]))
    return entries


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'requirements-cad.txt']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ] + data_tree('examples'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='axbis-jhjung',
    maintainer_email='scarlet777735@gmail.com',
    description='Work cell tools for fairino_sim',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'cell_manager = fairino_workcell.cell_manager:main',
            'import_cad = fairino_workcell.cad_import:main',
        ],
    },
)
