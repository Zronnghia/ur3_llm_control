from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ur3_llm_control'

setup(
    name=package_name,
    version='1.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Le Trong Nghia',
    maintainer_email='student@example.com',
    description='LLM-planned, MoveIt-executed pick & place for UR3/UR3e',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'llm_robot_node = ur3_llm_control.llm_robot_node:main',
            'send_command = ur3_llm_control.send_command:main',
            'check_scene = ur3_llm_control.check_scene:main',
            'offline_cli = ur3_llm_control.offline_cli:main',
        ],
    },
)
