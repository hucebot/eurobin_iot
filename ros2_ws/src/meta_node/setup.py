import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'meta_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*_launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='node that receive information from capteurs',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'listener = meta_node.my_node:main',
            'receiver = meta_node.other_node:main',
        ],
    },
)
