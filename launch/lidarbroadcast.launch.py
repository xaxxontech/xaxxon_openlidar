import os
from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	pkg = get_package_share_directory('xaxxon_openlidar')
	return LaunchDescription([
		Node(
			package='xaxxon_openlidar',
			executable='lidarbroadcast',
			name='lidarbroadcast',
			parameters=[os.path.join(pkg, 'default_params.yaml')],
			output='screen',
			# arguments=[('--ros-args --log-level DEBUG')],
			emulate_tty=True,
		)
	])
