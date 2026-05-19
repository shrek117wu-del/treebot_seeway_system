from setuptools import find_packages, setup

package_name = 'juxiedrive'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Seeway Development Team',
    author_email='shrek117wu@gmail.com',
    maintainer='Seeway Development Team',
    maintainer_email='shrek117wu@gmail.com',
    url='https://github.com/shrek117wu-del/treebot_seeway_system',
    keywords=['ROS2', 'Robot', 'CAN', 'CANFD', 'Joint Driver', 'JuxieDrive'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python :: 3',
    ],
    entry_points={
        'console_scripts': [
            'juxiedrive_node = juxiedrive.juxiedrive_node:main',
        ],
    },
)
