# launch/dialog_system.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='whisper_stt',
            executable='transcription_node',
            name='transcription_node',
            output='screen',
            parameters=[{
                "model_name": "small.en",
                "device": "cuda:0",
                "realtime_seconds": 3.0,
                "rate": 16000,
                "chunk": 1024,
                "channels": 1,
                "vad_mode": 3,
                "vad_speech_ratio": 0.3,
                "similarity_threshold": 80.0
            }]
        ),
        Node(
            package='ai_dialog_manager',
            executable='transcription_classificator_node',
            name='transcription_classificator_node',
            output='screen',
            parameters=[{
                "namespace": "ollama_interface",
                "service_name": "run_prompt",
                "transcription_topic": "/transcription_classifier/new_task"
            }]
        ),
        Node(
            package='task_extractor',
            executable='task_extractor_node',
            name='task_extractor_node',
            output='screen'
        ),
        Node(
            package='ai_dialog_manager',
            executable='dialog_manager_node',
            name='dialog_manager_node',
            output='screen'
        ),
        Node(
            package='ros2_ollama_interface',
            executable='llm_service_node',
            name='llm_service_node',
            output='screen'
        ),
    ])
