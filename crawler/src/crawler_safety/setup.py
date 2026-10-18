from setuptools import find_packages, setup

package_name = 'crawler_safety'

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
    description='Cliff mapping, keepout zones and motor current protection',
    license='GPL-3.0-or-later',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
                "cliff_keepout_node = crawler_safety.cliff_keepout_node:main",
                "cliff_costmap_node = crawler_safety.cliff_costmap_node:main",
                "current_guard_node = crawler_safety.current_guard_node:main",
        ],
    },
)
