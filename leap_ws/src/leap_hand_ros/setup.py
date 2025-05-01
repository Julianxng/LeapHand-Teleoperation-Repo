from setuptools import setup

package_name = 'leap_hand_ros'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(where='.', include=['leap_hand', 'leap_hand.*']),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Teleoperation system for the LEAP robotic hand using Ultraleap Motion Controller.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tcp_leap_receiver = leap_hand.tcp_leap_receiver:main',
            'leaphand_node = leap_hand.leaphand_node:main',
        ],
    },
)
