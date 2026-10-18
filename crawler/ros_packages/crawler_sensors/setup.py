from setuptools import find_packages, setup

package_name = 'crawler_sensors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md', 'LICENSE']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Christian Arrer',
    maintainer_email='EMAIL_ADDRESS_TO_BE_ADDED@example.invalid',
    description='ROS 2 I2C sensor drivers for Crawler Pi',
    license='GPL-3.0-only',
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
