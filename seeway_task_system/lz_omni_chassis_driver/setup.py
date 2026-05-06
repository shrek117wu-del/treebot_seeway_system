from setuptools import setup, find_packages

package_name = 'lz_omni_chassis_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/chassis_driver.launch.py']),
        ('share/' + package_name + '/config', ['config/chassis_driver_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Seeway Development Team',
    author_email='shrek117wu@gmail.com',
    maintainer='Seeway Development Team',
    maintainer_email='shrek117wu@gmail.com',
    url='https://github.com/shrek117wu-del/treebot_seeway_system',
    keywords=['ROS2', 'Robot', 'Chassis', 'Driver', 'LZ_OMNI'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
    ],
    entry_points={
        'console_scripts': [
            'chassis_driver_node = lz_omni_chassis_driver.chassis_driver_node:main',
        ],
    },
)
