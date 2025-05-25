#!/usr/bin/env python3
"""
그리퍼 확실히 수정된 브릿지
"""

import rospy
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64MultiArray
import socket
import json
import threading
import math
import numpy as np

# OVR2ROSInputs 메시지 정의
class OVR2ROSInputs:
    def __init__(self):
        self.button_upper = False
        self.button_lower = False
        self.thumb_stick_horizontal = 0.0
        self.thumb_stick_vertical = 0.0
        self.press_index = 0.0
        self.press_middle = 0.0

class GripperFixedBridge:
    def __init__(self):
        rospy.init_node('gripper_fixed_bridge')
        
        # 소켓 서버
        self.server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.server.bind(('0.0.0.0', 12345))
        self.server.listen(5)
        print("그리퍼 수정 브릿지 - 포트 12345")
        
        # 관절 데이터
        self.joints = [0.0, -0.3, 0.6, 0.0,    # 왼쪽
                      0.0, -0.3, 0.6, 0.0,      # 오른쪽
                      -0.005, -0.005]           # 그리퍼
        
        # 그리퍼 상태
        self.gripper_state = {
            'left': {'pressed': False, 'value': -0.005},
            'right': {'pressed': False, 'value': -0.005}
        }
        
        # VR 데이터
        self.vr_current = {
            'left': {'pos': None},
            'right': {'pos': None}
        }
        
        self.vr_init = {
            'left': {'pos': None},
            'right': {'pos': None}
        }
        
        self.clients = []
        self.lock = threading.Lock()
        
        # 파라미터
        self.position_scale = 3.0
        
        # ROS 구독
        rospy.Subscriber('/q2r_left_hand_pose', PoseStamped, 
                        lambda msg: self.vr_pose_callback(msg, 'left'))
        rospy.Subscriber('/q2r_right_hand_pose', PoseStamped, 
                        lambda msg: self.vr_pose_callback(msg, 'right'))
        
        # OVR2ROSInputs 구독 - 타입 명시
        try:
            from quest2ros.msg import OVR2ROSInputs
            rospy.Subscriber('/q2r_left_hand_inputs', OVR2ROSInputs, 
                            lambda msg: self.input_callback(msg, 'left'))
            rospy.Subscriber('/q2r_right_hand_inputs', OVR2ROSInputs, 
                            lambda msg: self.input_callback(msg, 'right'))
            print("✅ quest2ros.msg.OVR2ROSInputs 사용")
        except:
            # AnyMsg로 폴백
            rospy.Subscriber('/q2r_left_hand_inputs', rospy.AnyMsg, 
                            lambda msg: self.input_callback_any(msg, 'left'))
            rospy.Subscriber('/q2r_right_hand_inputs', rospy.AnyMsg, 
                            lambda msg: self.input_callback_any(msg, 'right'))
            print("⚠️  AnyMsg 폴백 사용")
        
        # 퍼블리셔
        self.pub = rospy.Publisher('/robot_joint_angles', Float64MultiArray, queue_size=1)
        
        # 업데이트 스레드
        threading.Thread(target=self.update_thread, daemon=True).start()
        
        # 키보드 스레드
        threading.Thread(target=self.keyboard_thread, daemon=True).start()
        
        print("\n=== 그리퍼 수정 브릿지 ===")
    
    def input_callback(self, msg, side):
        """OVR2ROSInputs 직접 처리"""
        with self.lock:
            # 어떤 버튼/트리거든 누르면 그리퍼 닫기
            pressed = (msg.button_upper or 
                      msg.button_lower or 
                      msg.press_index > 0.5 or
                      msg.press_middle > 0.5)
            
            if pressed != self.gripper_state[side]['pressed']:
                print(f"{side} 그리퍼: {'닫기' if pressed else '열기'}")
                if pressed:
                    print(f"  - button_upper: {msg.button_upper}")
                    print(f"  - button_lower: {msg.button_lower}")
                    print(f"  - press_index: {msg.press_index:.2f}")
                    print(f"  - press_middle: {msg.press_middle:.2f}")
            
            self.gripper_state[side]['pressed'] = pressed
    
    def input_callback_any(self, msg, side):
        """AnyMsg 파싱 - 더 정확한 방법"""
        try:
            # 버퍼에서 메시지 파싱
            from quest2ros.msg import OVR2ROSInputs
            input_msg = OVR2ROSInputs()
            input_msg.deserialize(msg._buff)
            
            # 이후 처리는 동일
            self.input_callback(input_msg, side)
            
        except:
            # 문자열 파싱 폴백
            try:
                lines = str(msg).split('\n')
                
                button_upper = False
                button_lower = False
                press_index = 0.0
                press_middle = 0.0
                
                for line in lines:
                    line = line.strip()
                    if line.startswith('button_upper:'):
                        button_upper = ('True' in line)
                    elif line.startswith('button_lower:'):
                        button_lower = ('True' in line)
                    elif line.startswith('press_index:'):
                        try:
                            press_index = float(line.split(':')[1].strip())
                        except:
                            pass
                    elif line.startswith('press_middle:'):
                        try:
                            press_middle = float(line.split(':')[1].strip())
                        except:
                            pass
                
                # 그리퍼 상태 업데이트
                with self.lock:
                    pressed = (button_upper or button_lower or 
                              press_index > 0.5 or press_middle > 0.5)
                    
                    if pressed != self.gripper_state[side]['pressed']:
                        print(f"{side} 그리퍼: {'닫기' if pressed else '열기'}")
                        if pressed:
                            print(f"  파싱 결과: upper={button_upper}, lower={button_lower}, index={press_index:.2f}")
                    
                    self.gripper_state[side]['pressed'] = pressed
                    
            except Exception as e:
                print(f"파싱 오류: {e}")
    
    def vr_pose_callback(self, msg, side):
        """VR 포즈 콜백"""
        pos = np.array([msg.pose.position.x, 
                       msg.pose.position.y, 
                       msg.pose.position.z])
        
        with self.lock:
            if self.vr_init[side]['pos'] is None:
                self.vr_init[side]['pos'] = pos.copy()
                print(f"{side} 초기화")
                return
            
            self.vr_current[side]['pos'] = pos
    
    def compute_control(self, side):
        """직접 제어"""
        if self.vr_current[side]['pos'] is None or self.vr_init[side]['pos'] is None:
            return
        
        delta = (self.vr_current[side]['pos'] - self.vr_init[side]['pos']) * self.position_scale
        dx, dy, dz = delta
        
        base_idx = 0 if side == 'left' else 4
        
        # 관절 매핑
        self.joints[base_idx] = np.clip(dx * 2.0, -3.14, 3.14)
        self.joints[base_idx + 1] = np.clip(-0.3 + dy * 2.0, -1.5, 1.5)
        self.joints[base_idx + 2] = np.clip(0.6 + dz * 2.0, -1.5, 1.4)
        self.joints[base_idx + 3] = 0.0
    
    def update_thread(self):
        """메인 업데이트"""
        rate = rospy.Rate(30)
        
        while not rospy.is_shutdown():
            with self.lock:
                # 움직임 제어
                self.compute_control('left')
                self.compute_control('right')
                
                # 그리퍼 업데이트
                # 왼쪽
                target = 0.019 if self.gripper_state['left']['pressed'] else -0.005
                self.joints[8] += (target - self.joints[8]) * 0.3
                
                # 오른쪽
                target = 0.019 if self.gripper_state['right']['pressed'] else -0.005
                self.joints[9] += (target - self.joints[9]) * 0.3
            
            self.publish()
            rate.sleep()
    
    def keyboard_thread(self):
        """키보드 제어"""
        print("\n키보드:")
        print("1/2: 그리퍼 테스트")
        print("r: 리셋")
        print("s: 상태")
        
        while not rospy.is_shutdown():
            try:
                key = input().strip().lower()
                
                if key == '1':
                    with self.lock:
                        self.gripper_state['left']['pressed'] = \
                            not self.gripper_state['left']['pressed']
                        print(f"왼쪽 그리퍼 수동: {'닫기' if self.gripper_state['left']['pressed'] else '열기'}")
                elif key == '2':
                    with self.lock:
                        self.gripper_state['right']['pressed'] = \
                            not self.gripper_state['right']['pressed']
                        print(f"오른쪽 그리퍼 수동: {'닫기' if self.gripper_state['right']['pressed'] else '열기'}")
                elif key == 'r':
                    self.reset()
                elif key == 's':
                    self.print_status()
            except:
                pass
    
    def reset(self):
        """리셋"""
        with self.lock:
            self.joints = [0.0, -0.3, 0.6, 0.0, 0.0, -0.3, 0.6, 0.0, -0.005, -0.005]
            self.gripper_state = {
                'left': {'pressed': False, 'value': -0.005},
                'right': {'pressed': False, 'value': -0.005}
            }
        print("리셋 완료")
    
    def print_status(self):
        """상태"""
        print(f"\n=== 상태 ===")
        with self.lock:
            print(f"왼쪽 그리퍼: {self.joints[8]:.3f} ({'닫힘' if self.gripper_state['left']['pressed'] else '열림'})")
            print(f"오른쪽 그리퍼: {self.joints[9]:.3f} ({'닫힘' if self.gripper_state['right']['pressed'] else '열림'})")
    
    def publish(self):
        """발행"""
        msg = Float64MultiArray()
        with self.lock:
            msg.data = self.joints
        self.pub.publish(msg)
        
        # 소켓 전송
        data = {
            'left': {
                'joint_angles': self.joints[0:4],
                'gripper': self.joints[8]
            },
            'right': {
                'joint_angles': self.joints[4:8],
                'gripper': self.joints[9]
            }
        }
        
        json_data = json.dumps(data) + '\n'
        with self.lock:
            for client in self.clients[:]:
                try:
                    client.sendall(json_data.encode())
                except:
                    self.clients.remove(client)
    
    def accept_clients(self):
        """클라이언트 수락"""
        while True:
            try:
                client, addr = self.server.accept()
                print(f"MuJoCo 클라이언트 연결: {addr}")
                with self.lock:
                    self.clients.append(client)
            except:
                pass
    
    def run(self):
        """실행"""
        threading.Thread(target=self.accept_clients, daemon=True).start()
        
        print("\n=== 그리퍼 수정 브릿지 실행 중 ===")
        print("✅ OVR2ROSInputs 정확한 처리")
        print("✅ 모든 버튼/트리거 지원")
        print("✅ 1/2 키로 그리퍼 수동 테스트")
        
        rospy.spin()

if __name__ == "__main__":
    bridge = GripperFixedBridge()
    bridge.run()

