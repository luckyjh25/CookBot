#!/usr/bin/env python3

import rospy
import socket
import json
import time
import threading
import numpy as np
from std_msgs.msg import Float64MultiArray, Float64
from sensor_msgs.msg import JointState

# 소켓 서버 설정
HOST = '0.0.0.0'  # 모든 인터페이스에서 연결 허용
PORT = 9090       # 사용할 포트

class SimplifiedBridgeNode:
    def __init__(self):
        # ROS1 노드 초기화
        rospy.init_node('ros1_socket_bridge')
        
        # IK 솔버에서 계산된 조인트 값 구독
        rospy.Subscriber("/robot_arm_left_joint", Float64MultiArray, self.left_joint_callback)
        rospy.Subscriber("/robot_arm_right_joint", Float64MultiArray, self.right_joint_callback)
        
        # 햅틱 피드백 구독 (IK 솔버에서 발행)
        rospy.Subscriber("/robot_arm_left_haptic", Float64, self.left_haptic_callback)
        rospy.Subscriber("/robot_arm_right_haptic", Float64, self.right_haptic_callback)
        
        # ROS1에서 조인트 상태도 발행 (선택사항)
        self.left_joints_pub = rospy.Publisher('/left_arm_joints', JointState, queue_size=10)
        self.right_joints_pub = rospy.Publisher('/right_arm_joints', JointState, queue_size=10)
        
        # 소켓 서버 시작
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(5)
        
        # 클라이언트 연결 관리
        self.clients = []
        
        # 서버 스레드 시작
        self.server_thread = threading.Thread(target=self.accept_clients)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        rospy.loginfo(f"Simplified ROS1 Socket Bridge started on port {PORT}")
    
    def accept_clients(self):
        """클라이언트 연결 수락 및 처리 스레드 시작"""
        while not rospy.is_shutdown():
            try:
                client, address = self.server_socket.accept()
                rospy.loginfo(f"Client connected: {address}")
                self.clients.append(client)
                
                # 클라이언트 처리 스레드 시작
                client_thread = threading.Thread(target=self.handle_client, args=(client, address))
                client_thread.daemon = True
                client_thread.start()
            except Exception as e:
                rospy.logerr(f"Error accepting client: {e}")
    
    def handle_client(self, client, address):
        """클라이언트와의 통신 처리 (역방향 햅틱 피드백 등)"""
        while not rospy.is_shutdown():
            try:
                data = client.recv(4096)
                if not data:
                    break
                
                # 현재는 단방향 전송만 하므로 특별한 처리 없음
                # 필요시 ROS2에서 ROS1로의 명령 처리 가능
                
            except Exception as e:
                rospy.logerr(f"Error handling client {address}: {e}")
                break
        
        # 연결 종료 처리
        try:
            client.close()
            self.clients.remove(client)
            rospy.loginfo(f"Client disconnected: {address}")
        except:
            pass
    
    def send_to_all_clients(self, message):
        """모든 클라이언트에 메시지 전송"""
        disconnected = []
        
        for client in self.clients:
            try:
                json_msg = json.dumps(message) + '\n'
                client.sendall(json_msg.encode('utf-8'))
            except:
                disconnected.append(client)
        
        # 연결 끊긴 클라이언트 제거
        for client in disconnected:
            try:
                client.close()
                self.clients.remove(client)
            except:
                pass
    
    def left_joint_callback(self, data):
        """왼팔 조인트 값 콜백 (IK 솔버에서 계산된 값)"""
        try:
            # IK 솔버에서 계산된 조인트 값: [j1, j2, j3, j4, gripper]
            joint_values = list(data.data)
            
            rospy.loginfo_throttle(2, f"Left joints from IK: {joint_values}")
            
            # ROS1 JointState도 발행 (선택사항)
            self.publish_joint_state(joint_values, 'left')
            
            # ROS2로 전송할 소켓 메시지
            message = {
                'type': 'joint_state',
                'side': 'left',
                'joint_names': [
                    'left_joint1', 
                    'left_joint2', 
                    'left_joint3', 
                    'left_joint4', 
                    'left_gripper_joint'
                ],
                'joint_positions': joint_values,
                'timestamp': time.time()
            }
            
            # 모든 클라이언트에 전송
            self.send_to_all_clients(message)
            
        except Exception as e:
            rospy.logerr(f"Error in left_joint_callback: {e}")
    
    def right_joint_callback(self, data):
        """오른팔 조인트 값 콜백 (IK 솔버에서 계산된 값)"""
        try:
            # IK 솔버에서 계산된 조인트 값: [j1, j2, j3, j4, gripper]
            joint_values = list(data.data)
            
            rospy.loginfo_throttle(2, f"Right joints from IK: {joint_values}")
            
            # ROS1 JointState도 발행 (선택사항)
            self.publish_joint_state(joint_values, 'right')
            
            # ROS2로 전송할 소켓 메시지
            message = {
                'type': 'joint_state',
                'side': 'right',
                'joint_names': [
                    'right_joint1', 
                    'right_joint2', 
                    'right_joint3', 
                    'right_joint4', 
                    'right_gripper_joint'
                ],
                'joint_positions': joint_values,
                'timestamp': time.time()
            }
            
            # 모든 클라이언트에 전송
            self.send_to_all_clients(message)
            
        except Exception as e:
            rospy.logerr(f"Error in right_joint_callback: {e}")
    
    def left_haptic_callback(self, data):
        """왼손 햅틱 피드백 콜백"""
        try:
            message = {
                'type': 'haptic_feedback',
                'side': 'left',
                'value': float(data.data),
                'timestamp': time.time()
            }
            
            self.send_to_all_clients(message)
            rospy.loginfo_throttle(5, f"Left haptic: {data.data}")
            
        except Exception as e:
            rospy.logerr(f"Error in left_haptic_callback: {e}")
    
    def right_haptic_callback(self, data):
        """오른손 햅틱 피드백 콜백"""
        try:
            message = {
                'type': 'haptic_feedback',
                'side': 'right',
                'value': float(data.data),
                'timestamp': time.time()
            }
            
            self.send_to_all_clients(message)
            rospy.loginfo_throttle(5, f"Right haptic: {data.data}")
            
        except Exception as e:
            rospy.logerr(f"Error in right_haptic_callback: {e}")
    
    def publish_joint_state(self, joint_values, side):
        """조인트 상태를 ROS1 JointState로도 발행 (선택사항)"""
        try:
            joint_state = JointState()
            joint_state.header.stamp = rospy.Time.now()
            
            if side == 'left':
                joint_state.name = [
                    'left_joint1', 'left_joint2', 'left_joint3', 
                    'left_joint4', 'left_gripper_joint'
                ]
                joint_state.position = joint_values
                self.left_joints_pub.publish(joint_state)
            else:
                joint_state.name = [
                    'right_joint1', 'right_joint2', 'right_joint3', 
                    'right_joint4', 'right_gripper_joint'
                ]
                joint_state.position = joint_values
                self.right_joints_pub.publish(joint_state)
                
        except Exception as e:
            rospy.logerr(f"Error publishing joint state: {e}")

if __name__ == '__main__':
    try:
        bridge = SimplifiedBridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 종료 시 소켓 정리
        try:
            bridge.server_socket.close()
        except:
            pass
