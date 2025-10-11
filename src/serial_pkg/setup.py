from setuptools import find_packages, setup
import os
import glob
package_name = 'serial_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc1',
    maintainer_email='rc1@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'serial_sender = serial_pkg.serial_sender:main',
            'serial_receiver = serial_pkg.serial_receiver:main',
            'fruit_arm_driver = serial_pkg.fruit_arm_driver:main',
            'data_merger_node = serial_pkg.merge_data:main',
            # 'manual_control_node = serial_pkg.manual_control_node:main',
            
        ],
    },
)
