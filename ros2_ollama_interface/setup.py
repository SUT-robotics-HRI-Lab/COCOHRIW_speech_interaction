from setuptools import find_packages, setup
import os

package_name = 'ros2_ollama_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(include=['ros2_ollama_interface', 'ros2_ollama_interface.*']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'langchain_ollama'],
    package_data={
        'ros2_ollama_interface': ['service/config.yaml','client/config.yaml'],
    },
    include_package_data=True,
    zip_safe=True,
    maintainer='marek',
    maintainer_email='marek.cornak@stuba.sk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        	'llm_service_node = ros2_ollama_interface.service.llm_service_node:main',
        	'test_client = ros2_ollama_interface.client.test_client:main',
        ],
    },
)
