from setuptools import setup

package_name = 'dynamic_obstacle_controller'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kakashi-hatake',
    maintainer_email='kakashi-hatake@todo.todo',
    description='Moves the dynamic obstacles in the Gazebo simulation.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'obstacle_controller = dynamic_obstacle_controller.obstacle_controller:main',
        ],
    },
)
