from setuptools import find_packages, setup

package_name = 'task_extractor'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'config.yaml']),
    ],
    install_requires=[
        'setuptools',
        'pydantic',
        'langchain',
        'langchain-ollama',
        'pyyaml',
    ],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='LLM-powered task extractor using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'task_extractor_node = task_extractor.task_extractor_node:main',
        ],
    },
)
