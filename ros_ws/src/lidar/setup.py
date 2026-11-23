from setuptools import find_packages, setup

package_name = 'lidar'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kakashi-hatake',
    maintainer_email='kakashi-hatake@todo.todo',
    description='Groups LiDAR points into obstacles and marks them as moving or static.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_detector = lidar.obstacle_detector:main',
        ],
    },
)
