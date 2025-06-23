from setuptools import setup

package_name = 'pure_pursuit'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='junhakim308',
    maintainer_email='junha308@gmail.com',
    description='Pure Pursuit Controller Node for ROS 2 (F1TENTH)',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pure_pursuit = pure_pursuit.pure_pursuit_node:main',
        ],
    },
)
