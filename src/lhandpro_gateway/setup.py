from glob import glob
from setuptools import find_packages, setup

package_name = 'lhandpro_gateway'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='maintainer',
    maintainer_email='maintainer@example.com',
    description='LHandPro teleoperation gateway skeleton.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_command_adapter = lhandpro_gateway.hand_command_adapter:main',
            'hand_command_adapter_dual = lhandpro_gateway.hand_command_adapter_dual:main',
            'hand_state_publisher = lhandpro_gateway.hand_state_publisher:main',
            'hand_state_publisher_single = lhandpro_gateway.hand_state_publisher_single:main',
        ],
    },
)
