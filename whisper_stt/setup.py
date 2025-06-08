from setuptools import find_packages, setup

package_name = 'whisper_stt'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'PyYAML',
        'ament_index_python',
    ],
    zip_safe=True,
    maintainer='marek',
    maintainer_email='marek.cornak@stuba.sk',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transcription_node = whisper_stt.transcription_node:main'
        ],
    },
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Install config.yaml to the share directory
        ('share/' + package_name + '/config', ['config/config.yaml']),
    ],
)
