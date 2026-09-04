from setuptools import setup
import os
from glob import glob
package_name = 'yahboomcar_description'


def package_tree(root):
    """Install nested mesh directories while preserving package:// paths."""
    entries = []
    for directory, _, _ in os.walk(root):
        files = glob(os.path.join(directory, '*.*'))
        if files:
            entries.append((os.path.join('share', package_name, directory), files))
    return entries

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share',package_name,'launch'),glob(os.path.join('launch','*launch.py'))),
        (os.path.join('share',package_name,'urdf'),glob(os.path.join('urdf','*.*'))),
        #(os.path.join('share',package_name,'meshes/Ackermann'),glob(os.path.join('meshes/Ackermann','*.*'))),
        #(os.path.join('share',package_name,'meshes/mecanum'),glob(os.path.join('meshes/mecanum','*.*'))),
        *package_tree('meshes'),
        (os.path.join('share',package_name,'rviz'),glob(os.path.join('rviz','*.rviz*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='nx-ros2',
    maintainer_email='nx-ros2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        "test": ["pytest"],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
