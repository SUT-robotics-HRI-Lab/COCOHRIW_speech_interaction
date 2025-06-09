from setuptools import setup, find_packages

package_name = 'ai_dialog_manager'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(include=[package_name, f"{package_name}.*"]),
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/transcription_classificator', [
            'ai_dialog_manager/transcription_classificator/config.yaml',
            'ai_dialog_manager/transcription_classificator/invalid_input.yaml',
        ]),
        ('share/ai_dialog_manager/launch', ['launch/dialog_system.launch.py']),
        ('share/ai_dialog_manager/launch', ['launch/llm_nodes.launch.py']),

        # Optional: include dialog manager configs if needed
        # ('share/' + package_name + '/dialog_manager', [
        #     'ai_dialog_manager/dialog_manager/config.yaml',
        # ]),
    ],
    install_requires=['setuptools', 'ament_index_python'],
    include_package_data=True,
    zip_safe=True,
    maintainer='marek',
    maintainer_email='marek.cornak@stuba.sk',
    description='AI-based transcription classification and dialog state manager',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'transcription_classificator_node = ai_dialog_manager.transcription_classificator.transcription_classificator_node:main',
            'dialog_manager_node = ai_dialog_manager.dialog_manager.dialog_manager_node:main',
        ],
    },
)