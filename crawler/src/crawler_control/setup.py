from setuptools import find_packages, setup

package_name = 'crawler_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'gradio'],
    author='ChatGPT and Christian Arrer',
    author_email='christianarrer@gmail.com',
    zip_safe=True,
    maintainer='Christian Arrer',
    maintainer_email='christianarrer@gmail.com',
    description='Tracked vehicle velocity, odometry, teleoperation and motor control',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'teleop = crawler_control.teleop_node:main',
        'velocity_target = crawler_control.velocity_target_node:main',
	'velocity_target_cal = crawler_control.velocity_target_cal_node:main',
        'motor_driver = crawler_control.motor_driver_node:main',
        'odom = crawler_control.odom_node:main',
        'odom_cal = crawler_control.odom_cal_node:main',
        ],
    },
)
