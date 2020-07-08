from setuptools import setup

package_name = 'xaxxon_openlidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/lidarbroadcast.launch.py']),
        ('share/' + package_name, ['default_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='colin adamson',
    maintainer_email='colin@xaxxon.com',
    description='ROS2 Drivers for the Xaxxon OpenLIDAR Sensor',
    license='GPLv3',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'broadcast = xaxxon_openlidar.lidarbroadcast:main',
        ],
    },
)
