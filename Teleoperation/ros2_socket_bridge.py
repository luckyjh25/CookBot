#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
import socket
import json
import threading
import time
import numpy as np

from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# 소켓 클라이언트 설정
HOST = '127.0.0.1'  # localhost
PORT = 9090         # 서버와 동일한 포트

class ROS2SocketBridge(Node):
    def __init__(self):
        super().__init__('ros2_socket_bridge')
        
        # ROS2 퍼블리셔 - 조인트 상태만 발행
        self.left_joints_pub = self.create_publisher(JointState, '/left_arm_joints', 10)
        self.right_joints_pub = self.create_publisher(JointState, '/right_arm_joints', 10)
        
        # ROS2 서브스크라이버 (ROS1으로 전송할 데이터 구독)
        self.left_haptic_sub = self.create_subscription(
            Float32, '/left_hand_haptic_feedback', self.left_haptic_callback, 10)
        self.right_haptic_sub = self.create_subscription(
            Float32, '/right_hand_haptic_feedback', self.right_haptic_callback, 10)
        
        # 버튼 상태 저장
        self.left_trigger_pressed = False
        self.right_trigger_pressed = False
        
        # 소켓 연결
        self.socket = None
        self.connected = False
        self.data_buffer = ""
        
        # 연결 시도
        self.connect_to_server()
        
        # 연결 유지 타이머
        self.create_timer(5.0, self.check_connection)
        
        self.get_logger().info('ROS2 Socket Bridge started')
    
    def connect_to_server(self):
        """서버에 연결 시도"""
        if self.connected:
            return
        
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((HOST, PORT))
            self.connected = True
            self.get_logger().info(f'Connected to ROS1 socket server at {HOST}:{PORT}')
            
            # 수신 스레드 시작
            self.receive_thread = threading.Thread(target=self.receive_data)
            self.receive_thread.daemon = True
            self.receive_thread.start()
            
        except Exception as e:
            self.get_logger().error(f'Failed to connect to server: {e}')
            self.connected = False
    
    def check_connection(self):
        """연결 상태 확인 및 필요시 재연결"""
        if not self.connected:
            self.get_logger().info('Attempting to reconnect...')
            self.connect_to_server()
    
    def receive_data(self):
        """서버로부터 데이터 수신"""
        while rclpy.ok() and self.connected:
            try:
                data = self.socket.recv(4096).decode('utf-8')
                if not data:
                    self.connected = False
                    self.get_logger().warn('Connection closed by server')
                    break
                
                # 데이터 버퍼에 추가
                self.data_buffer += data
                
                # 줄바꿈으로 구분된 메시지 처리
                while '\n' in self.data_buffer:
                    line, self.data_buffer = self.data_buffer.split('\n', 1)
                    try:
                        message = json.loads(line)
                        self.process_message(message)
                    except json.JSONDecodeError as e:
                        self.get_logger().warn(f'Received invalid JSON data: {e}')
                
            except Exception as e:
                self.get_logger().error(f'Error receiving data: {e}')
                self.connected = False
                break
    
    def process_message(self, message):
        """수신된 메시지 처리"""
        try:
            message_type = message.get('type')
            
            # 조인트 값 메시지 처리
            if message_type == 'joint_values':
                side = message.get('side')
                joint_values = message.get('joint_values')
                
                if side == 'left':
                    self.publish_left_joints(joint_values)
                elif side == 'right':
                    self.publish_right_joints(joint_values)
            
            # 버튼 상태 메시지 처리
            elif message_type == 'button_state':
                side = message.get('side')
                
                if side == 'left':
                    self.left_trigger_pressed = message.get('trigger_pressed', False)
                elif side == 'right':
                    self.right_trigger_pressed = message.get('trigger_pressed', False)
            
        except Exception as e:
            self.get_logger().error(f'Error processing message: {e}')
    
    def publish_left_joints(self, joint_values):
        """왼쪽 팔 조인트 상태 발행"""
        try:
            # JointState 메시지 생성
            msg = JointState()
            
            # 헤더 설정
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = ''
            
            # 조인트 이름 및 값 설정
            msg.name = [
                'left_joint1', 'left_joint2', 'left_joint3', 
                'left_joint4', 'left_gripper_joint'
            ]
            msg.position = joint_values
            
            # 발행
            self.left_joints_pub.publish(msg)
            self.get_logger().debug(f'Published left arm joints: {joint_values}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing left joints: {e}')
    
    def publish_right_joints(self, joint_values):
        """오른쪽 팔 조인트 상태 발행"""
        try:
            # JointState 메시지 생성
            msg = JointState()
            
            # 헤더 설정
            now = self.get_clock().now()
            msg.header.stamp = now.to_msg()
            msg.header.frame_id = ''
            
            # 조인트 이름 및 값 설정
            msg.name = [
                'right_joint1', 'right_joint2', 'right_joint3', 
                'right_joint4', 'right_gripper_joint'
            ]
            msg.position = joint_values
            
            # 발행
            self.right_joints_pub.publish(msg)
            self.get_logger().debug(f'Published right arm joints: {joint_values}')
            
        except Exception as e:
            self.get_logger().error(f'Error publishing right joints: {e}')
    
    def left_haptic_callback(self, msg):
        """왼쪽 컨트롤러 햅틱 피드백 콜백"""
        if not self.connected:
            return
        
        try:
            message = {
                'type': 'haptic_feedback',
                'side': 'left',
                'value': msg.data
            }
            
            self.socket.sendall(json.dumps(message).encode('utf-8') + b'\n')
            self.get_logger().debug(f'Sent left haptic feedback: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending haptic feedback: {e}')
            self.connected = False
    
    def right_haptic_callback(self, msg):
        """오른쪽 컨트롤러 햅틱 피드백 콜백"""
        if not self.connected:
            return
        
        try:
            message = {
                'type': 'haptic_feedback',
                'side': 'right',
                'value': msg.data
            }
            
            self.socket.sendall(json.dumps(message).encode('utf-8') + b'\n')
            self.get_logger().debug(f'Sent right haptic feedback: {msg.data}')
            
        except Exception as e:
            self.get_logger().error(f'Error sending haptic feedback: {e}')
            self.connected = False

def main(args=None):
    rclpy.init(args=args)
    node = ROS2SocketBridge()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.socket:
            try:
                node.socket.close()
            except:
                pass
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
