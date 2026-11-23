from setuptools import find_packages, setup

package_name = 'nmpc'

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
    description='Nonlinear MPC that makes the AGV follow the target while avoiding obstacles.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'nmpc_controller = nmpc.nmpc_node:main',
        ],
    },
)
