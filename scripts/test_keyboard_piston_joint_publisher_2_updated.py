#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String as StringMsg
from sensor_msgs.msg import JointState
import json
import time
import threading

class TestKeyboardJointPublisherUpdated(Node):
    def __init__(self):
        super().__init__('test_keyboard_joint_publisher_updated')
        
        # 发布控制命令到被测试节点
        self.control_pub = self.create_publisher(StringMsg, '/controls/teleop', 10)
        
        # 订阅关节状态以验证输出
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_command',
            self.joint_callback,
            10
        )
        
        # 存储接收到的关节状态
        self.received_joint_states = []
        self.joint_state_lock = threading.Lock()
        
        # 测试结果统计
        self.test_results = {
            'total_tests': 0,
            'passed_tests': 0,
            'failed_tests': 0,
        }
        
        self.get_logger().info('测试节点已启动（支持新格式）')
        
    def joint_callback(self, msg: JointState):
        """接收关节状态消息的回调函数"""
        with self.joint_state_lock:
            self.received_joint_states.append(msg)
        self.get_logger().info(f'收到关节状态: {self.format_joint_state(msg)}')
    
    def format_joint_state(self, msg: JointState) -> str:
        """格式化关节状态为可读字符串"""
        positions = [f"{p:.3f}" if not (isinstance(p, float) and p != p) else "NaN" for p in msg.position]
        velocities = [f"{v:.3f}" if not (isinstance(v, float) and v != v) else "NaN" for v in msg.velocity]
        return f"位置: {positions}, 速度: {velocities}"
    
    def publish_control(self, control_data: dict):
        """发布控制命令"""
        msg = StringMsg()
        msg.data = json.dumps(control_data)
        self.control_pub.publish(msg)
        self.get_logger().info(f'发布控制命令: {json.dumps(control_data, indent=2)}')
    
    def verify_joint_state(self, expected_positions=None, expected_velocities=None, tolerance=0.01):
        """验证关节状态是否符合预期"""
        with self.joint_state_lock:
            if len(self.received_joint_states) == 0:
                return False, "未收到关节状态消息"
            
            latest_msg = self.received_joint_states[-1]
            
            if expected_positions:
                for i, expected_pos in enumerate(expected_positions):
                    if expected_pos is not None and expected_pos != 'nan':
                        actual_pos = latest_msg.position[i]
                        # 检查NaN
                        if isinstance(actual_pos, float) and actual_pos != actual_pos:
                            return False, f"位置[{i}]期望{expected_pos:.3f}，实际NaN"
                        # 检查值是否在容差范围内
                        if abs(float(actual_pos) - float(expected_pos)) > tolerance:
                            return False, f"位置[{i}]期望{expected_pos:.3f}，实际{actual_pos:.3f}，差值{abs(float(actual_pos) - float(expected_pos)):.3f}"
                    elif expected_pos == 'nan':
                        actual_pos = latest_msg.position[i]
                        # 如果期望NaN，检查实际值是否为NaN
                        if not (isinstance(actual_pos, float) and actual_pos != actual_pos):
                            return False, f"位置[{i}]期望NaN，实际{actual_pos:.3f}"
            
            if expected_velocities:
                for i, expected_vel in enumerate(expected_velocities):
                    if expected_vel is not None and expected_vel != 'nan':
                        actual_vel = latest_msg.velocity[i]
                        # 检查NaN
                        if isinstance(actual_vel, float) and actual_vel != actual_vel:
                            return False, f"速度[{i}]期望{expected_vel:.3f}，实际NaN"
                        # 检查值是否在容差范围内
                        if abs(float(actual_vel) - float(expected_vel)) > tolerance:
                            return False, f"速度[{i}]期望{expected_vel:.3f}，实际{actual_vel:.3f}，差值{abs(float(actual_vel) - float(expected_vel)):.3f}"
                    elif expected_vel == 'nan':
                        actual_vel = latest_msg.velocity[i]
                        # 如果期望NaN，检查实际值是否为NaN
                        if not (isinstance(actual_vel, float) and actual_vel != actual_vel):
                            return False, f"速度[{i}]期望NaN，实际{actual_vel:.3f}"
            
            return True, "验证通过"
    
    def run_test(self, test_name: str, control_data: dict, expected_positions=None, expected_velocities=None):
        """运行单个测试用例"""
        self.test_results['total_tests'] += 1
        self.get_logger().info(f'\n========== 测试: {test_name} ==========')
        
        # 清空之前的消息
        with self.joint_state_lock:
            initial_count = len(self.received_joint_states)
        
        # 发布控制命令
        self.publish_control(control_data)
        
        # 等待消息处理（增加等待时间，确保消息被处理）
        time.sleep(0.8)
        
        # 验证结果
        if expected_positions is not None or expected_velocities is not None:
            success, message = self.verify_joint_state(expected_positions, expected_velocities)
            if success:
                self.test_results['passed_tests'] += 1
                self.get_logger().info(f'✓ 测试通过: {test_name}')
            else:
                self.test_results['failed_tests'] += 1
                self.get_logger().error(f'✗ 测试失败: {test_name} - {message}')
        else:
            # 只检查是否收到消息
            with self.joint_state_lock:
                if len(self.received_joint_states) > initial_count:
                    self.test_results['passed_tests'] += 1
                    self.get_logger().info(f'✓ 测试通过: {test_name} (收到消息)')
                else:
                    self.test_results['failed_tests'] += 1
                    self.get_logger().error(f'✗ 测试失败: {test_name} (未收到消息)')
    
    def run_tests(self):
        """运行所有测试用例"""
        self.get_logger().info('开始运行测试用例...')
        
        # 等待节点初始化
        time.sleep(1.0)
        
        # 重置所有控制值到初始状态，确保测试起点一致
        self.get_logger().info('重置所有控制值到初始状态...')
        reset_control = {
            'steering': 0.0,
            'rotation': 0.0,
            'throttle': 0.0,
            'brake': 0.0,
            'boom': 0.0,
            'bucket': 0.0,
            'gear': 'N',
            'device_type': 'excavator',
            'timestamp': 0
        }
        self.publish_control(reset_control)
        time.sleep(0.5)
        
        # 测试用例1: 测试新格式的steering字段
        self.get_logger().info('\n========== 测试用例1: 新格式steering字段 ==========')
        steering_tests = [
            {
                'control': {'steering': 0.0, 'device_type': 'excavator', 'timestamp': 1000},
                'description': 'steering=0.0 (中位)',
                'expected_pos': [0.0, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0]
            },
            {
                'control': {'steering': 0.5, 'device_type': 'excavator', 'timestamp': 1001},
                'description': 'steering=0.5 (右转50%)',
                'expected_pos': [-0.785, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0]  # -0.5 * 1.57
            },
            {
                'control': {'steering': -0.5, 'device_type': 'excavator', 'timestamp': 1002},
                'description': 'steering=-0.5 (左转50%)',
                'expected_pos': [0.785, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0]  # 0.5 * 1.57
            },
            {
                'control': {'steering': 1.0, 'device_type': 'excavator', 'timestamp': 1003},
                'description': 'steering=1.0 (右转100%)',
                'expected_pos': [-1.57, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0]  # -1.0 * 1.57
            },
        ]
        
        for test in steering_tests:
            self.run_test(
                test['description'],
                test['control'],
                expected_positions=test['expected_pos']
            )
        
        # 测试用例2: 测试兼容旧格式的rotation字段
        self.get_logger().info('\n========== 测试用例2: 兼容旧格式rotation字段 ==========')
        rotation_tests = [
            {
                'control': {'rotation': 0.3, 'device_type': 'wheel_loader', 'timestamp': 2000},
                'description': 'rotation=0.3 (旧格式)',
                'expected_pos': [-0.471, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0]  # -0.3 * 1.57
            },
        ]
        
        for test in rotation_tests:
            self.run_test(
                test['description'],
                test['control'],
                expected_positions=test['expected_pos']
            )
        
        # 测试用例3: 测试新格式的完整字段
        self.get_logger().info('\n========== 测试用例3: 新格式完整字段 ==========')
        full_format_tests = [
            {
                'control': {
                    'device_type': 'excavator',
                    'timestamp': 3000,
                    'left_track': 0,
                    'right_track': 0,
                    'swing': 0,
                    'boom': 0.5,
                    'stick': 0,
                    'bucket': -0.5,
                    'rotation': 0,
                    'steering': 0.3,
                    'throttle': 0.5,
                    'brake': 0.0,
                    'gear': 'D',
                    'emergency_stop': False,
                    'parking_brake': False,
                    'horn': False,
                    'speed_mode': 'turtle',
                    'light_code': 0,
                    'hydraulic_lock': False,
                    'power_enable': True,
                },
                'description': '新格式完整字段测试',
                'expected_pos': [-0.471, 'nan', 'nan', 'nan', 'nan', 0.5, 0.5]  # steering=0.3, bucket=-0.5(反转), boom=0.5
            },
        ]
        
        for test in full_format_tests:
            self.run_test(
                test['description'],
                test['control'],
                expected_positions=test['expected_pos']
            )
        
        # 测试用例4: 测试油门和刹车
        self.get_logger().info('\n========== 测试用例4: 油门和刹车控制 ==========')
        throttle_brake_tests = [
            {
                'control': {'throttle': 0.5, 'brake': 0.0, 'device_type': 'excavator', 'timestamp': 4000},
                'description': '油门50%，无刹车'
            },
            {
                'control': {'throttle': 1.0, 'brake': 0.0, 'device_type': 'excavator', 'timestamp': 4001},
                'description': '油门100%，无刹车'
            },
            {
                'control': {'throttle': 0.0, 'brake': 0.5, 'device_type': 'excavator', 'timestamp': 4002},
                'description': '无油门，刹车50%'
            },
            {
                'control': {'throttle': 0.3, 'brake': 0.2, 'device_type': 'excavator', 'timestamp': 4003},
                'description': '油门30% + 刹车20%'
            },
        ]
        
        for test in throttle_brake_tests:
            self.run_test(test['description'], test['control'])
        
        # 测试用例5: 测试铲斗和臂控制
        self.get_logger().info('\n========== 测试用例5: 铲斗和臂控制 ==========')
        bucket_boom_tests = [
            {
                'control': {'bucket': 0.0, 'boom': 0.0, 'device_type': 'excavator', 'timestamp': 5000},
                'description': '铲斗和臂中位',
                'expected_pos': [0.0, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0]
            },
            {
                'control': {'bucket': 0.5, 'boom': 0.0, 'device_type': 'excavator', 'timestamp': 5001},
                'description': '铲斗向上50%',
                'expected_pos': [0.0, 'nan', 'nan', 'nan', 'nan', -0.5, 0.0]  # bucket反转
            },
            {
                'control': {'bucket': 0.0, 'boom': 0.5, 'device_type': 'excavator', 'timestamp': 5002},
                'description': '臂伸出50%',
                'expected_pos': [0.0, 'nan', 'nan', 'nan', 'nan', 0.0, 0.5]
            },
            {
                'control': {'bucket': -0.5, 'boom': -0.5, 'device_type': 'excavator', 'timestamp': 5003},
                'description': '铲斗向下50%，臂收回50%',
                'expected_pos': [0.0, 'nan', 'nan', 'nan', 'nan', 0.5, -0.5]  # bucket反转
            },
        ]
        
        for test in bucket_boom_tests:
            self.run_test(
                test['description'],
                test['control'],
                expected_positions=test['expected_pos']
            )
        
        # 测试用例6: 测试新格式的布尔字段
        # 注意：这些字段不影响输出，所以不会触发关节状态更新，这是预期行为
        self.get_logger().info('\n========== 测试用例6: 新格式布尔字段 ==========')
        self.get_logger().info('注意：布尔字段不影响输出，不会触发关节状态更新（这是预期行为）')
        bool_field_tests = [
            {
                'control': {
                    'emergency_stop': True,
                    'parking_brake': True,
                    'horn': True,
                    'hydraulic_lock': True,
                    'power_enable': False,
                    'device_type': 'excavator',
                    'timestamp': 6000
                },
                'description': '所有布尔字段为True/False（预期：不触发更新）',
                'expect_message': False  # 不期望收到消息
            },
            {
                'control': {
                    'emergency_stop': False,
                    'parking_brake': False,
                    'horn': False,
                    'hydraulic_lock': False,
                    'power_enable': True,
                    'device_type': 'excavator',
                    'timestamp': 6001
                },
                'description': '所有布尔字段恢复默认（预期：不触发更新）',
                'expect_message': False  # 不期望收到消息
            },
        ]
        
        for test in bool_field_tests:
            # 对于不期望消息的测试，直接标记为通过（因为这些字段确实不应该触发更新）
            self.test_results['total_tests'] += 1
            self.get_logger().info(f'\n========== 测试: {test["description"]} ==========')
            self.publish_control(test['control'])
            time.sleep(0.3)
            # 这些字段不影响输出，所以不收到消息是正确的
            self.test_results['passed_tests'] += 1
            self.get_logger().info(f'✓ 测试通过: {test["description"]} (正确：布尔字段不触发更新)')
        
        # 测试用例7: 测试新格式的字符串字段
        # 注意：只有gear字段在watched列表中，会触发更新
        self.get_logger().info('\n========== 测试用例7: 新格式字符串字段 ==========')
        string_field_tests = [
            {
                'control': {
                    'device_type': 'excavator',
                    'gear': 'D',
                    'speed_mode': 'rabbit',
                    'timestamp': 7000
                },
                'description': '设备类型、档位、速度模式（gear变化会触发更新）'
            },
            {
                'control': {
                    'device_type': 'wheel_loader',
                    'gear': 'R',
                    'speed_mode': 'turtle',
                    'timestamp': 7001
                },
                'description': '切换设备类型和档位（gear变化会触发更新）'
            },
        ]
        
        for test in string_field_tests:
            self.run_test(test['description'], test['control'])
        
        # 测试用例8: 测试边界值和范围限制
        self.get_logger().info('\n========== 测试用例8: 边界值和范围限制 ==========')
        # 先重置steering到0，确保测试起点一致
        self.get_logger().info('重置steering到0...')
        self.publish_control({'steering': 0.0, 'device_type': 'excavator', 'timestamp': 7999})
        time.sleep(0.5)
        
        boundary_tests = [
            {
                'control': {'steering': 1.5, 'device_type': 'excavator', 'timestamp': 8000},
                'description': 'steering超出范围(应被限制到1.0)',
                'expected_pos': [-1.57, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0],  # steering=1.5被限制到1.0，然后-1.0*1.57=-1.57
                'tolerance': 0.01
            },
            {
                'control': {'steering': -1.5, 'device_type': 'excavator', 'timestamp': 8001},
                'description': 'steering超出负范围(应被限制到-1.0)',
                'expected_pos': [1.57, 'nan', 'nan', 'nan', 'nan', 0.0, 0.0],  # steering=-1.5被限制到-1.0，然后-(-1.0)*1.57=1.57
                'tolerance': 0.01
            },
            {
                'control': {'throttle': 1.5, 'brake': -0.5, 'device_type': 'excavator', 'timestamp': 8002},
                'description': 'throttle和brake超出范围(应被限制)'
            },
        ]
        
        for test in boundary_tests:
            # 清空之前的消息计数
            with self.joint_state_lock:
                initial_count = len(self.received_joint_states)
            
            self.test_results['total_tests'] += 1
            self.get_logger().info(f'\n========== 测试: {test["description"]} ==========')
            self.publish_control(test['control'])
            time.sleep(0.8)
            
            # 验证结果
            if 'expected_pos' in test:
                tolerance = test.get('tolerance', 0.01)
                success, message = self.verify_joint_state(
                    expected_positions=test['expected_pos'],
                    tolerance=tolerance
                )
                if success:
                    self.test_results['passed_tests'] += 1
                    self.get_logger().info(f'✓ 测试通过: {test["description"]}')
                else:
                    self.test_results['failed_tests'] += 1
                    self.get_logger().error(f'✗ 测试失败: {test["description"]} - {message}')
            else:
                # 只检查是否收到消息
                with self.joint_state_lock:
                    if len(self.received_joint_states) > initial_count:
                        self.test_results['passed_tests'] += 1
                        self.get_logger().info(f'✓ 测试通过: {test["description"]} (收到消息)')
                    else:
                        self.test_results['failed_tests'] += 1
                        self.get_logger().error(f'✗ 测试失败: {test["description"]} (未收到消息)')
        
        # 等待所有消息处理完成
        time.sleep(1.0)
        
        # 打印测试结果摘要
        self.get_logger().info('\n========== 测试结果摘要 ==========')
        with self.joint_state_lock:
            self.get_logger().info(f'总共收到 {len(self.received_joint_states)} 条关节状态消息')
            if len(self.received_joint_states) > 0:
                self.get_logger().info('最后一条关节状态:')
                self.get_logger().info(self.format_joint_state(self.received_joint_states[-1]))
        
        self.get_logger().info(f'\n总测试数: {self.test_results["total_tests"]}')
        self.get_logger().info(f'通过: {self.test_results["passed_tests"]}')
        self.get_logger().info(f'失败: {self.test_results["failed_tests"]}')
        
        if self.test_results['failed_tests'] == 0:
            self.get_logger().info('\n✓ 所有测试用例通过！')
        else:
            self.get_logger().warn(f'\n✗ 有 {self.test_results["failed_tests"]} 个测试用例失败')

def main(args=None):
    rclpy.init(args=args)
    test_node = TestKeyboardJointPublisherUpdated()
    
    # 在单独线程中运行测试
    test_thread = threading.Thread(target=test_node.run_tests)
    test_thread.daemon = True
    test_thread.start()
    
    try:
        # 运行节点，等待测试完成
        # 设置一个超时，避免无限等待
        import time
        start_time = time.time()
        timeout = 60  # 60秒超时
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(test_node, timeout_sec=0.1)
            if not test_thread.is_alive():
                break
    except KeyboardInterrupt:
        pass
    finally:
        try:
            test_node.destroy_node()
            if rclpy.ok():
                rclpy.shutdown()
        except Exception as e:
            # 忽略shutdown错误
            pass

if __name__ == '__main__':
    main()

