from setuptools import find_packages, setup
import os
import glob
package_name = 'camera_controller'

# 递归收集models目录下的所有文件
def get_model_files():
    data_files = []
    for root, dirs, files in os.walk('models'):
        if files:
            rel_dir = os.path.relpath(root, '.')
            install_dir = os.path.join('share', package_name, rel_dir)
            file_paths = [os.path.join(root, f) for f in files]
            data_files.append((install_dir, file_paths))
    return data_files

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob.glob('config/*')),
    ] + get_model_files(),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rc1',
    maintainer_email='edmounds@163.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fruit_detect = camera_controller.fruit_detect:main',
        ],
    }
)

