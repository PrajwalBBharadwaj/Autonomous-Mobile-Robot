from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
	return LaunchDescription([
		Node(
			package='kurma_object_follower',
			executable='image_process',
			name='image_processing'
		),
		Node(
			package='kurma_object_follower',
			executable='rotate_robot',
			name='rotate_robot'
		),


	])
