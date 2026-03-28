from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='go2_audio_perception',
            executable='audio_perception_node',
            name='audio_perception_node',
            parameters=[{
                'energy_threshold': 500.0,
                'publish_rate_hz': 10.0
            }]
        ),
        Node(
            package='go2_audio_perception',
            executable='nemo_asr_node',
            name='nemo_asr_node',
            parameters=[{
                'model_name': 'nvidia/stt_en_fastconformer_ctc_small'
            }]
        )
    ])
