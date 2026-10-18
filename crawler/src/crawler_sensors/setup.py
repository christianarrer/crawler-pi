from setuptools import find_packages, setup

package_name = 'crawler_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    author='ChatGPT and Christian Arrer',
    author_email='christianarrer@gmail.com',
    zip_safe=True,
    maintainer='Christian Arrer',
    maintainer_email='christianarrer@gmail.com',
    description='I2C IMU, battery monitoring and magnetic odometry sensors',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'ina226_node = crawler_sensors.ina226_node:main',
            'tmag_odom_pair_node = crawler_sensors.tmag_odom_pair_node:main',
            'icm20948_node = crawler_sensors.icm20948_node:main',
        ],
    },
)
