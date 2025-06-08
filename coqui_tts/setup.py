from setuptools import setup
import os

package_name = 'coqui_tts'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        # Required ROS 2 index entries
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'coqui_tts/config.yaml']),

        # Include the config.yaml file manually
        #('share/' + package_name + '/coqui_tts', ['coqui_tts/config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='User',
    maintainer_email='user@example.com',
    description='TTS service node with parameters loaded from config.yaml',
    license='MIT',
    entry_points={
        'console_scripts': [
            'tts_service_node = coqui_tts.tts_service_node:main',
        ],
    },
)
