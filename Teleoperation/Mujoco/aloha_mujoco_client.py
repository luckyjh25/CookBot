#!/usr/bin/env python3
"""
ALOHA 모델용 MuJoCo 클라이언트
dual_arm_robot.xml과 호환되도록 수정
"""

import socket
import json
import time
import threading
import numpy as np
import mujoco
import mujoco.viewer

class ALOHAMuJoCoClient:
    def __init__(self, model_path, server_host='localhost', server_port=12345):
        self.server_host = server_host
        self.server_port = server_port
        self.model_path = model_path
        self.running = False
        self.lock = threading.Lock()
        self.data_received = False
        self.connected = False
        
        # 통합 관절 데이터 (브릿지 형식에 맞춤)
        self.joint_data = {
            'left': {
                'joint_angles': [0.0, -0.3, 0.6, 0.0],
                'gripper': -0.005
            },
            'right': {
                'joint_angles': [0.0, -0.3, 0.6, 0.0],
                'gripper': -0.005
            }
        }
        
        # MuJoCo 모델 로드
        print(f"MuJoCo 모델 로드 중: {self.model_path}")
        self.model = mujoco.MjModel.from_xml_path(self.model_path)
        self.data = mujoco.MjData(self.model)
        
        # 액추에이터 이름 (dual_arm_robot.xml 기준)
        self.actuator_names = {
            'left': [
                "left_actuator_joint1",
                "left_actuator_joint2", 
                "left_actuator_joint3",
                "left_actuator_joint4",
                "left_actuator_gripper_joint"
            ],
            'right': [
                "right_actuator_joint1",
                "right_actuator_joint2",
                "right_actuator_joint3",
                "right_actuator_joint4",
                "right_actuator_gripper_joint"
            ]
        }
        
        # 액추에이터 인덱스 매핑
        self.actuator_indices = self._map_actuators()
        
        # 초기 위치 설정
        self.reset_position()
    
    def _map_actuators(self):
        """액추에이터 인덱스 매핑"""
        indices = {'left': [], 'right': []}
        
        print("\n=== 액추에이터 매핑 ===")
        
        # 사용 가능한 모든 액추에이터 출력
        print("사용 가능한 액추에이터:")
        for i in range(self.model.nu):
            act_name = mujoco.mj_id2name(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
            print(f"  [{i}] {act_name}")
        
        # 액추에이터 매핑
        for side in ['left', 'right']:
            for name in self.actuator_names[side]:
                try:
                    idx = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_ACTUATOR, name)
                    if idx >= 0:
                        indices[side].append(idx)
                        print(f"✅ {name}: 인덱스 {idx}")
                    else:
                        indices[side].append(-1)
                        print(f"❌ {name}: 찾을 수 없음")
                except:
                    indices[side].append(-1)
                    print(f"❌ {name}: 오류")
        
        return indices
    
    def connect_to_server(self):
        """서버에 연결"""
        try:
            print(f"\n서버 {self.server_host}:{self.server_port}에 연결 중...")
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.server_host, self.server_port))
            self.socket.settimeout(0.1)
            print("✅ 서버 연결 성공!")
            self.connected = True
            return True
        except Exception as e:
            print(f"❌ 서버 연결 실패: {e}")
            self.connected = False
            return False
    
    def receive_data(self):
        """서버로부터 데이터 수신"""
        self.running = True
        buffer = ""
        
        while self.running:
            try:
                chunk = self.socket.recv(4096).decode()
                if not chunk:
                    print("서버 연결 종료")
                    self.connected = False
                    break
                
                buffer += chunk
                
                # JSON 데이터 파싱
                while '\n' in buffer:
                    line, buffer = buffer.split('\n', 1)
                    try:
                        data = json.loads(line)
                        with self.lock:
                            # 브릿지 형식 데이터 처리
                            if 'left' in data and 'right' in data:
                                self.joint_data = data
                                self.data_received = True
                                
                                # 디버깅 출력 (가끔)
                                if np.random.random() < 0.05:  # 5% 확률
                                    left_angles = [f"{np.degrees(a):.0f}°" for a in data['left']['joint_angles']]
                                    right_angles = [f"{np.degrees(a):.0f}°" for a in data['right']['joint_angles']]
                                    print(f"수신: L={left_angles}, R={right_angles}")
                    
                    except json.JSONDecodeError:
                        pass
            
            except socket.timeout:
                pass
            except Exception as e:
                if self.running:
                    print(f"데이터 수신 오류: {e}")
                break
        
        print("데이터 수신 스레드 종료")
    
    def update_robot_joints(self):
        """수신된 관절 각도로 로봇 업데이트"""
        if not self.data_received:
            return
        
        with self.lock:
            # 왼쪽 팔
            left_data = self.joint_data.get('left', {})
            if 'joint_angles' in left_data:
                for i, angle in enumerate(left_data['joint_angles']):
                    if i < len(self.actuator_indices['left']) and self.actuator_indices['left'][i] >= 0:
                        self.data.ctrl[self.actuator_indices['left'][i]] = angle
                
                # 그리퍼
                if len(self.actuator_indices['left']) > 4 and self.actuator_indices['left'][4] >= 0:
                    self.data.ctrl[self.actuator_indices['left'][4]] = left_data.get('gripper', -0.005)
            
            # 오른쪽 팔
            right_data = self.joint_data.get('right', {})
            if 'joint_angles' in right_data:
                for i, angle in enumerate(right_data['joint_angles']):
                    if i < len(self.actuator_indices['right']) and self.actuator_indices['right'][i] >= 0:
                        self.data.ctrl[self.actuator_indices['right'][i]] = angle
                
                # 그리퍼
                if len(self.actuator_indices['right']) > 4 and self.actuator_indices['right'][4] >= 0:
                    self.data.ctrl[self.actuator_indices['right'][4]] = right_data.get('gripper', -0.005)
    
    def reset_position(self):
        """로봇을 초기 위치로 리셋"""
        # 키프레임 확인
        keyframe_id = mujoco.mj_name2id(self.model, mujoco.mjtObj.mjOBJ_KEY, "neutral_pose")
        if keyframe_id >= 0:
            mujoco.mj_resetDataKeyframe(self.model, self.data, keyframe_id)
            print("✅ neutral_pose 키프레임으로 리셋")
        else:
            # 수동 리셋
            default_angles = [0.0, -0.3, 0.6, 0.0]
            
            # 왼쪽 팔
            for i, angle in enumerate(default_angles):
                if i < len(self.actuator_indices['left']) and self.actuator_indices['left'][i] >= 0:
                    self.data.ctrl[self.actuator_indices['left'][i]] = angle
            if len(self.actuator_indices['left']) > 4 and self.actuator_indices['left'][4] >= 0:
                self.data.ctrl[self.actuator_indices['left'][4]] = -0.005
            
            # 오른쪽 팔
            for i, angle in enumerate(default_angles):
                if i < len(self.actuator_indices['right']) and self.actuator_indices['right'][i] >= 0:
                    self.data.ctrl[self.actuator_indices['right'][i]] = angle
            if len(self.actuator_indices['right']) > 4 and self.actuator_indices['right'][4] >= 0:
                self.data.ctrl[self.actuator_indices['right'][4]] = -0.005
            
            print("✅ 수동으로 초기 위치 설정")
    
    def test_joint_movement(self):
        """관절 움직임 테스트"""
        print("\n=== 관절 움직임 테스트 ===")
        
        # 왼쪽 joint1 테스트
        if self.actuator_indices['left'][0] >= 0:
            print("왼쪽 Joint1 회전 테스트...")
            for angle in [-1.0, 0.0, 1.0, 0.0]:
                self.data.ctrl[self.actuator_indices['left'][0]] = angle
                mujoco.mj_step(self.model, self.data)
                time.sleep(0.5)
                print(f"  각도: {np.degrees(angle):.0f}°")
        
        # 오른쪽 joint1 테스트
        if self.actuator_indices['right'][0] >= 0:
            print("오른쪽 Joint1 회전 테스트...")
            for angle in [-1.0, 0.0, 1.0, 0.0]:
                self.data.ctrl[self.actuator_indices['right'][0]] = angle
                mujoco.mj_step(self.model, self.data)
                time.sleep(0.5)
                print(f"  각도: {np.degrees(angle):.0f}°")
        
        self.reset_position()
        print("테스트 완료!")
    
    def run(self):
        """클라이언트 실행"""
        # 서버 연결 시도
        if self.connect_to_server():
            # 데이터 수신 스레드 시작
            data_thread = threading.Thread(target=self.receive_data)
            data_thread.daemon = True
            data_thread.start()
        else:
            print("⚠️  서버 없이 실행합니다. T키로 테스트 가능")
        
        # MuJoCo 시뮬레이션 및 뷰어
        with mujoco.viewer.launch_passive(self.model, self.data) as viewer:
            # 카메라 설정
            viewer.cam.distance = 1.5
            viewer.cam.azimuth = 90
            viewer.cam.elevation = -20
            
            print("\n=== ALOHA MuJoCo 클라이언트 ===")
            print("R: 초기 위치로 리셋")
            print("T: Joint1 회전 테스트")
            print("S: 현재 상태 출력")
            print("ESC: 종료")
            print("마우스: 카메라 제어")
            
            last_update_time = time.time()
            
            # 메인 루프
            while viewer.is_running() and self.running:
                current_time = time.time()
                
                # 관절 업데이트
                self.update_robot_joints()
                
                # 시뮬레이션 스텝
                mujoco.mj_step(self.model, self.data)
                
                # 뷰어 업데이트
                viewer.sync()
                
                # 상태 출력 (1초마다)
                if current_time - last_update_time > 1.0:
                    last_update_time = current_time
                    if self.connected and self.data_received:
                        # 가끔 상태 출력
                        if np.random.random() < 0.1:  # 10% 확률
                            print(f"연결 상태: 서버={self.connected}, 데이터={self.data_received}")
                
                # 프레임 레이트 제어
                time.sleep(0.01)
        
        self.running = False
        
        # 소켓 닫기
        if hasattr(self, 'socket'):
            try:
                self.socket.close()
            except:
                pass
        
        print("\n클라이언트 종료")

def main():
    import sys
    
    # 모델 경로 설정 (명령줄 인자 또는 기본값)
    if len(sys.argv) > 1:
        model_path = sys.argv[1]
    else:
        # 기본 경로들 시도
        possible_paths = [
            "/home/ss2928/mujoco_ws/src/CookingBot/Mujoco/dual_arm_robot.xml",
            "/home/ss2928/mujoco_ws/src/CookingBot/aloha/scene.xml",
            "dual_arm_robot.xml",
            "aloha/scene.xml"
        ]
        
        model_path = None
        for path in possible_paths:
            try:
                with open(path, 'r'):
                    model_path = path
                    break
            except:
                continue
        
        if not model_path:
            print("❌ 모델 파일을 찾을 수 없습니다!")
            print("사용법: python3 aloha_mujoco_client.py [모델_경로]")
            return
    
    # 서버 설정
    server_host = "localhost"  # 또는 "192.168.0.61"
    
    print(f"모델 파일: {model_path}")
    print(f"서버 주소: {server_host}:12345")
    
    client = ALOHAMuJoCoClient(model_path=model_path, server_host=server_host)
    
    # 뷰어에서 키보드 콜백 추가
    def on_key(key):
        if key == ord('R') or key == ord('r'):
            client.reset_position()
            print("초기 위치로 리셋")
        elif key == ord('T') or key == ord('t'):
            client.test_joint_movement()
        elif key == ord('S') or key == ord('s'):
            print("\n=== 현재 상태 ===")
            print(f"연결: {client.connected}")
            print(f"데이터 수신: {client.data_received}")
            if client.data_received:
                with client.lock:
                    left = client.joint_data.get('left', {})
                    right = client.joint_data.get('right', {})
                    if 'joint_angles' in left:
                        print(f"왼쪽: {[f'{np.degrees(a):.0f}°' for a in left['joint_angles']]}")
                    if 'joint_angles' in right:
                        print(f"오른쪽: {[f'{np.degrees(a):.0f}°' for a in right['joint_angles']]}")
    
    client.run()

if __name__ == "__main__":
    main()
