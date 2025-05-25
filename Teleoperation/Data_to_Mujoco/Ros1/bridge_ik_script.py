#!/usr/bin/env python3

import rospy
import socket
import json
import time
import threading
import numpy as np
from geometry_msgs.msg import PoseStamped, Twist
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float32

# 소켓 서버 설정
HOST = '0.0.0.0'  # 모든 인터페이스에서 연결 허용
PORT = 9090       # 사용할 포트

class BridgeNode:
    def __init__(self):
        # ROS1 노드 초기화
        rospy.init_node('ros1_socket_bridge')
        
        # 구독할 토픽
        rospy.Subscriber("/q2r_left_hand_pose", PoseStamped, self.left_pose_callback)
        rospy.Subscriber("/q2r_left_hand_inputs", Joy, self.left_inputs_callback)
        rospy.Subscriber("/q2r_right_hand_pose", PoseStamped, self.right_pose_callback)
        rospy.Subscriber("/q2r_right_hand_inputs", Joy, self.right_inputs_callback)
        
        # 햅틱 피드백용 발행자
        self.left_haptic_pub = rospy.Publisher('/q2r_left_hand_haptic_feedback', Float32, queue_size=10)
        self.right_haptic_pub = rospy.Publisher('/q2r_right_hand_haptic_feedback', Float32, queue_size=10)
        
        # 조인트 상태 발행자
        self.left_joints_pub = rospy.Publisher('/left_arm_joints', JointState, queue_size=10)
        self.right_joints_pub = rospy.Publisher('/right_arm_joints', JointState, queue_size=10)
        
        # 소켓 서버 시작
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server_socket.bind((HOST, PORT))
        self.server_socket.listen(5)
        
        # 클라이언트 연결 관리
        self.clients = []
        
        # 조인트 스케일링 파라미터
        self.pos_scale_x = 20.0  # 좌우 움직임 (x축)
        self.pos_scale_y = 15.0  # 전후 움직임 (y축)
        self.pos_scale_z = 15.0  # 상하 움직임 (z축)
        self.orientation_scale = 5.0  # 방향 민감도
        
        # 서버 스레드 시작
        self.server_thread = threading.Thread(target=self.accept_clients)
        self.server_thread.daemon = True
        self.server_thread.start()
        
        rospy.loginfo(f"ROS1 Socket Bridge started on port {PORT}")
    
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
        """클라이언트와의 통신 처리"""
        while not rospy.is_shutdown():
            try:
                data = client.recv(4096)
                if not data:
                    break
                
                # 데이터 파싱 및 처리
                message = json.loads(data.decode('utf-8'))
                message_type = message.get('type')
                
                # 햅틱 피드백 처리
                if message_type == 'haptic_feedback':
                    side = message.get('side', 'left')
                    value = float(message.get('value', 0.0))
                    
                    if side == 'left':
                        self.left_haptic_pub.publish(value)
                    elif side == 'right':
                        self.right_haptic_pub.publish(value)
                
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
                client.sendall(json.dumps(message).encode('utf-8') + b'\n')  # newline 추가
            except:
                disconnected.append(client)
        
        # 연결 끊긴 클라이언트 제거
        for client in disconnected:
            try:
                client.close()
                self.clients.remove(client)
            except:
                pass
    
    def quaternion_to_euler(self, qx, qy, qz, qw):
        """쿼터니언을 오일러 각도로 변환"""
        # 롤 (x-axis 회전)
        sinr_cosp = 2 * (qw * qx + qy * qz)
        cosr_cosp = 1 - 2 * (qx * qx + qy * qy)
        roll = np.arctan2(sinr_cosp, cosr_cosp)
        
        # 피치 (y-axis 회전)
        sinp = 2 * (qw * qy - qz * qx)
        if abs(sinp) >= 1:
            pitch = np.sign(sinp) * np.pi / 2  # 90 degrees if out of range
        else:
            pitch = np.arcsin(sinp)
        
        # 요 (z-axis 회전)
        siny_cosp = 2 * (qw * qz + qx * qy)
        cosy_cosp = 1 - 2 * (qy * qy + qz * qz)
        yaw = np.arctan2(siny_cosp, cosy_cosp)
        
        return roll, pitch, yaw
    
    def map_to_joint_angles(self, pose_data, side, trigger_pressed=False):
        """위치 및 방향 데이터를 5개의 조인트 값으로 매핑"""
        # 데이터 추출
        px = pose_data.pose.position.x
        py = pose_data.pose.position.y
        pz = pose_data.pose.position.z
        qx = pose_data.pose.orientation.x
        qy = pose_data.pose.orientation.y
        qz = pose_data.pose.orientation.z
        qw = pose_data.pose.orientation.w
        
        # 오일러 각도 계산
        roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
        
        # 조인트 각도 계산
        joint_values = [0.0] * 5
        
        # 허리 관절 (x 위치 기반, 좌우 움직임)
        joint_values[0] = px * self.pos_scale_x
        
        # 어깨 관절 (z 위치 기반, 상하 움직임)
        joint_values[1] = -0.3 - pz * self.pos_scale_z
        
        # 팔꿈치 관절 (y 위치 기반, 전후 움직임)
        joint_values[2] = 0.6 + py * self.pos_scale_y
        
        # 손목 관절 (피치 회전 기반)
        joint_values[3] = pitch * self.orientation_scale
        
        # 그리퍼 (트리거 버튼 기반)
        joint_values[4] = 0.015 if trigger_pressed else -0.005
        
        # 관절 제한
        joint_values[0] = np.clip(joint_values[0], -3.14, 3.14)  # 허리
        joint_values[1] = np.clip(joint_values[1], -1.5, 1.5)    # 어깨
        joint_values[2] = np.clip(joint_values[2], -1.5, 1.4)    # 팔꿈치
        joint_values[3] = np.clip(joint_values[3], -1.7, 1.97)   # 손목
        joint_values[4] = np.clip(joint_values[4], -0.01, 0.02)  # 그리퍼
        
        return joint_values
    
    def publish_joint_state(self, joint_values, side):
        """조인트 상태 발행"""
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
    
    def left_pose_callback(self, data):
        """왼쪽 컨트롤러 위치 데이터 처리"""
        try:
            # 트리거 상태 확인 (기본값 False)
            trigger_pressed = False
            
            # 조인트 각도 계산
            joint_values = self.map_to_joint_angles(data, 'left', trigger_pressed)
            
            # ROS 조인트 상태 발행
            self.publish_joint_state(joint_values, 'left')
            
            # 소켓 메시지 생성
            message = {
                'type': 'joint_values',
                'side': 'left',
                'joint_values': joint_values,
                'timestamp': {
                    'secs': data.header.stamp.secs,
                    'nsecs': data.header.stamp.nsecs
                }
            }
            
            # 클라이언트에 전송
            self.send_to_all_clients(message)
            
        except Exception as e:
            rospy.logerr(f"Error in left_pose_callback: {e}")
    
    def left_inputs_callback(self, data):
        """왼쪽 컨트롤러 입력 데이터 처리"""
        try:
            # 버튼 상태 업데이트
            # 주의: 버튼 인덱스는 실제 Quest 2 컨트롤러 매핑에 따라 조정 필요
            trigger_pressed = data.buttons[0] == 1  # 예시: 첫 번째 버튼이 트리거라고 가정
            grip_pressed = data.buttons[1] == 1     # 예시: 두 번째 버튼이 그립이라고 가정
            x_pressed = data.buttons[2] == 1        # X 버튼
            y_pressed = data.buttons[3] == 1        # Y 버튼
            
            # 버튼 정보 소켓 메시지
            message = {
                'type': 'button_state',
                'side': 'left',
                'trigger_pressed': trigger_pressed,
                'grip_pressed': grip_pressed,
                'x_pressed': x_pressed,
                'y_pressed': y_pressed
            }
            
            # 클라이언트에 전송
            self.send_to_all_clients(message)
            
        except Exception as e:
            rospy.logerr(f"Error in left_inputs_callback: {e}")
    
    def right_pose_callback(self, data):
        """오른쪽 컨트롤러 위치 데이터 처리"""
        try:
            # 트리거 상태 확인 (기본값 False)
            trigger_pressed = False
            
            # 조인트 각도 계산
            joint_values = self.map_to_joint_angles(data, 'right', trigger_pressed)
            
            # ROS 조인트 상태 발행 
            self.publish_joint_state(joint_values, 'right')
            
            # 소켓 메시지 생성
            message = {
                'type': 'joint_values',
                'side': 'right',
                'joint_values': joint_values,
                'timestamp': {
                    'secs': data.header.stamp.secs,
                    'nsecs': data.header.stamp.nsecs
                }
            }
            
            # 클라이언트에 전송
            self.send_to_all_clients(message)
            
        except Exception as e:
            rospy.logerr(f"Error in right_pose_callback: {e}")
    
    def right_inputs_callback(self, data):
        """오른쪽 컨트롤러 입력 데이터 처리"""
        try:
            # 버튼 상태 업데이트
            # 주의: 버튼 인덱스는 실제 Quest 2 컨트롤러 매핑에 따라 조정 필요
            trigger_pressed = data.buttons[0] == 1  # 예시: 첫 번째 버튼이 트리거라고 가정
            grip_pressed = data.buttons[1] == 1     # 예시: 두 번째 버튼이 그립이라고 가정
            a_pressed = data.buttons[2] == 1        # A 버튼
            b_pressed = data.buttons[3] == 1        # B 버튼
            
            # 버튼 정보 소켓 메시지
            message = {
                'type': 'button_state',
                'side': 'right',
                'trigger_pressed': trigger_pressed,
                'grip_pressed': grip_pressed,
                'a_pressed': a_pressed,
                'b_pressed': b_pressed
            }
            
            # 클라이언트에 전송
            self.send_to_all_clients(message)
            
        except Exception as e:
            rospy.logerr(f"Error in right_inputs_callback: {e}")

if __name__ == '__main__':
    try:
        bridge = BridgeNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        # 종료 시 소켓 정리
        try:
            bridge.server_socket.close()
        except:
            pass
