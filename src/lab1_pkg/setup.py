from setuptools import find_packages, setup
import os

package_name = 'lab1_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        #installing the launch files and config files to the share directory
        (os.path.join('share', package_name, 'launch'), ['launch/talker_relay.launch.py']),
        (os.path.join('share', package_name, 'config'), ['config/params.yaml'])
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='kanishka',
    maintainer_email='varshinikanishka@iitgn.ac.in',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    #executables: example command: ros2 run lab1_pkg test --ros-args -p v:=1.0 -p d:=2.3
    entry_points={
        'console_scripts': [                      
        'test = lab1_pkg.py_node:main',
        'talker = lab1_pkg.talker:main',
        'relay = lab1_pkg.relay:main',
        'safety= lab1_pkg.safety:main'
        ],
    },
)
