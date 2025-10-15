from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'task_decision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc1',
    maintainer_email='edmounds@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'task_decision_node = task_decision.task_decision:main',
        ],
    },
)
