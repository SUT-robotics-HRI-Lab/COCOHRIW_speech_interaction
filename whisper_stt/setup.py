#!/home/marek/eurbin_ws/venv/bin/python3
from setuptools import find_packages, setup

package_name = 'whisper_stt'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    py_modules=[],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Github: MarekC96',
    maintainer_email='marek.cornak@stuba.sk',
    description='A ROS2 package for real-time audio transcription using Whisper and validation of speeech using local llm model with Ollama',
    license='MIT',
    entry_points={
        'console_scripts': [
            'transcription_node = whisper_stt.transcription_node:main',
        ],
    },
)
