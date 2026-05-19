from setuptools import find_packages, setup

package_name = 'juxiedrive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/juxiedrive.launch.py']),
        ('share/' + package_name + '/config', ['config/juxiedrive_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Seeway Development Team',
    author_email='shrek117wu@gmail.com',
    maintainer='Seeway Development Team',
    maintainer_email='shrek117wu@gmail.com',
    url='https://github.com/shrek117wu-del/treebot_seeway_system',
    keywords=['ROS2', 'CAN', 'CAN FD', 'CANopen', 'Joint', 'Actuator'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
    ],
    entry_points={
        'console_scripts': [
            'driver_node = juxiedrive.driver_node:main',
        ],
    },
)
