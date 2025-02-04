from setuptools import find_packages, setup

package_name = 'twist2tf2_pkg_g01'

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
    maintainer='ivan',
    maintainer_email='ivan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'twist2tf2_node_g01= twist2tf2_pkg_g01.twist2tf2_node_g01:main',
            ],
    },
)
