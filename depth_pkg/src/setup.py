from setuptools import find_packages, setup

package_name = 'depth_pkg'

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
    maintainer='Tegbir Lalli',
    maintainer_email='teglal28@gmail.com',
    description='ROS2 to MAVLink bridge',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'depth_to_mavlink = depth_pkg.depth_pkg:main',
        ],
    },
)
