from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='Teleop2CanTransformer',
            executable='teleop2can_transformer',
            name='teleop2can_transformer',
            output='screen',
            parameters=[{
                # 死区参数
                'steering_deadzone': 0.05,
                'throttle_deadzone': 0.05,
                'brake_deadzone': 0.05,
                'boom_deadzone': 0.05,
                'bucket_deadzone': 0.05,
                
                # 角度映射范围（度）
                'arm_angle_min': -60.0,
                'arm_angle_max': 60.0,
                'shovel_angle_min': -60.0,
                'shovel_angle_max': 60.0,
                
                # 速度限制（m/s）
                'max_speed': 3.0,
            }]
        )
    ])
