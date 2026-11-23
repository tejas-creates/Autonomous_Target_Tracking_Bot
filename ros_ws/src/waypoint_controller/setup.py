from setuptools import setup

package_name = 'waypoint_controller'

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
    description='Acceleration-limited waypoint controller for Target_AGV.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'waypoint_controller = waypoint_controller.waypoint_controller:main',
        ],
    },
)
