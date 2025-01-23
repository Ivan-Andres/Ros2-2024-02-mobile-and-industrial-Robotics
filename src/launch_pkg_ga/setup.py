from setuptools import find_packages, setup
# librerias para el sistema operativo, y poder movernos entre carptas
from glob import glob
import os

package_name = 'launch_pkg_ga'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    # agregamos la carpeta de launch al setup
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share", package_name,'launch'), glob("launch/*.launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ivan',
    maintainer_email='ivan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
