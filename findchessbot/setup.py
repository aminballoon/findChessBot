from setuptools import setup
import os
from glob import glob

package_name = 'findchessbot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
         (os.path.join('share', package_name), glob('launch/*.launch.py')),
         (os.path.join('share', package_name), glob('urdf/*')),
         (os.path.join('share', package_name), glob('meshes/*')),
         (os.path.join('share', package_name), glob('rviz/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='bally',
    maintainer_email='bally@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pub = findchessbot.pub:main',
            'sub = findchessbot.sub:main',
            'input_to_jointstate = findchessbot.input_to_jointstate:main',
            'state_publisher = findchessbot.state_publisher:main',
        ],
    },
)
