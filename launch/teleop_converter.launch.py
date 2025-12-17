from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # 默认参数文件（使用已安装的包路径）
    pkg_share = get_package_share_directory('teleoptocantransformer')
    default_params = os.path.join(pkg_share, 'config', 'output_topics.yaml')

    return LaunchDescription([
        Node(
            package='teleoptocantransformer',
            executable='teleop2can_transformer',
            name='teleop2can_transformer',
            output='screen',
            parameters=[
                {
                    # 死区参数
                    'steering_deadzone': 0.05,
                    'throttle_deadzone': 0.05,
                    'brake_deadzone': 0.05,
                    'boom_deadzone': 0.05,
                    'bucket_deadzone': 0.05,
                    
                    # 角度映射范围（度）
                    # 大臂范围：-800~800
                    # 铲斗范围：-800~800
                    'arm_angle_min': -800.0,
                    'arm_angle_max': 800.0,
                    'shovel_angle_min': -800.0,
                    'shovel_angle_max': 800.0,
                    
                    # 速度限制（m/s）
                    'max_speed': 3.0,
                },
                # 额外参数文件（含 topic 发布开关等）
                default_params
            ]
        )
    ])
