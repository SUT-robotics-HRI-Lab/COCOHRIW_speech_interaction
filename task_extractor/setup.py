from setuptools import setup

package_name = 'task_extractor'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools','task_msgs'],
    zip_safe=True,
    maintainer='MarekC96',
    maintainer_email='marek.cornak@stuba.sk',
    description='Task extraction package for ROS 2',
    license='MIT',
    entry_points={
        'console_scripts': [
            'task_validator_node = task_extractor.task_validator_node:main', 
        ],
    },
)
