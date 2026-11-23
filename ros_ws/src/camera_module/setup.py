from setuptools import find_packages, setup

package_name = 'camera_module'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    tests_require=['pytest'],
    zip_safe=True,
    maintainer='kakashi-hatake',
    maintainer_email='kakashi-hatake@todo.todo',
    description='Simple checkerboard camera detector for the simulated AGV.',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'camera_module = camera_module.camera_module:main',
        ],
    },
)
