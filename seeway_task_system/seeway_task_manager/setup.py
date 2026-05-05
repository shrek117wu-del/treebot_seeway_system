from setuptools import setup, find_packages

package_name = 'seeway_task_manager'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author='Seeway Development Team',
    author_email='seeway@example.com',
    maintainer='Seeway Development Team',
    maintainer_email='seeway@example.com',
    url='https://github.com/shrek117wu-del/treebot_seeway_system',
    download_url='https://github.com/shrek117wu-del/treebot_seeway_system',
    keywords=['ROS2', 'Robot', 'Task Management'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
    ],
    entry_points={
        'console_scripts': [
            'task_manager_node = seeway_task_manager.task_manager_node:main',
            'xbox_controller_node = seeway_task_manager.xbox_controller_node:main',
            'nav2_client_node = seeway_task_manager.nav2_client_node:main',
            'task_scheduler_node = seeway_task_manager.task_scheduler_node:main',
        ],
    },
)
