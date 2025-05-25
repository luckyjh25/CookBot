#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32MultiArray  # haptic 메시지 타입 (실제 타입에 맞게 수정 필요)
import serial
import json
import time

class LeftArmBridge(Node):
    def __init__(self):
        super().__init__('left_arm_bridge')
        
        # 시리얼 포트 설정
        try:
            self.serial_port = serial.Serial('/dev/ttyACM0', 57600, timeout=1)
            time.sleep(2)
            self.get_logger().info('Serial connection established')
        except Exception as e:
            self.get_logger().error(f'Serial connection failed: {e}')
            return
        
        # left_arm_joints 토픽 구독
        self.left_arm_subscription = self.create_subscription(
            JointState,
            'left_arm_joints',
            self.left_arm_callback,
            10)
            
        # left_hand_haptic_feedback 토픽 구독
        self.left_haptic_subscription = self.create_subscription(
            Float32MultiArray,  # 실제 메시지 타입으로 변경 필요
            'left_hand_haptic_feedback',
            self.left_haptic_callback,
            10)
        
        self.get_logger().info('Left Arm Bridge Node Started - Listening to left_arm_joints and left_hand_haptic_feedback')
    
    def left_arm_callback(self, msg):
        try:
            # JointState 메시지를 JSON으로 변환
            joint_data = {
                'names': list(msg.name),
                'positions': list(msg.position),
                'velocities': list(msg.velocity) if msg.velocity else [],
                'efforts': list(msg.effort) if msg.effort else []
            }
            
            # 시리얼로 전송
            data_str = f"LEFT_ARM_JOINTS:{json.dumps(joint_data)}\n"
            self.serial_port.write(data_str.encode())
            
            self.get_logger().info(f'Sent left_arm_joints: {len(msg.position)} joints')
            
        except Exception as e:
            self.get_logger().error(f'Left arm send error: {e}')
    
    def left_haptic_callback(self, msg):
        try:
            # Haptic 메시지를 JSON으로 변환 (메시지 구조에 따라 수정 필요)
            haptic_data = {
                'data': list(msg.data) if hasattr(msg, 'data') else str(msg)
            }
            
            # 시리얼로 전송
            data_str = f"LEFT_HAPTIC:{json.dumps(haptic_data)}\n"
            self.serial_port.write(data_str.encode())
            
            self.get_logger().info('Sent left_hand_haptic_feedback')
            
        except Exception as e:
            self.get_logger().error(f'Left haptic send error: {e}')

def main():
    rclpy.init()
    bridge = LeftArmBridge()
    try:
        rclpy.spin(bridge)
    except KeyboardInterrupt:
        pass
    finally:
        bridge.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
