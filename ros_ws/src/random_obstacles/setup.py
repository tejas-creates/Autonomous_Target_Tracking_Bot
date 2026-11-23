from setuptools import find_packages, setup

package_name = 'random_obstacles'

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
    description='Moves the garden robots (Moving_Cylinder_*): random +/-45 deg legs or route crossings.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'random_obstacle_controller = random_obstacles.random_obstacle_controller:main',
        ],
    },
)
