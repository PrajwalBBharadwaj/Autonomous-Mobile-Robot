from setuptools import setup
import os
from glob import glob
package_name = 'kurma_object_follower'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),(os.path.join('share', package_name), glob('launch/*.launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dexter',
    maintainer_email='akrishnan331@gatech.edu','pbharadwaj33@gatech.edu'
    description='Test package to complete lab2 ECE7785',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["image_publisher=kurma_object_follower.camera_node:main","image_process=kurma_object_follower.image_processing:main","rotate_robot=kurma_object_follower.rotate_robot:main"],
	}
	)
