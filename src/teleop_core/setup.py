from glob import glob
from setuptools import find_packages, setup

package_name = 'teleop_core'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='Teleoperation mapping and supervision skeleton.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vr_to_arm_mapper = teleop_core.vr_to_arm_mapper:main',
            'vr_to_hand_mapper = teleop_core.vr_to_hand_mapper:main',
        ],
    },
)
