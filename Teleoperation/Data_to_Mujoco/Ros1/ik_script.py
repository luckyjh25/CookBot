#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import PoseStamped
from quest2ros.msg import OVR2ROSInputs
from std_msgs.msg import Float64, Float64MultiArray
import tf.transformations as tf_trans

class Quest2ROSOptimizedIK:
    def __init__(self):
        rospy.init_node('quest2ros_optimized_ik', anonymous=True)
        
        # VR 데이터 저장
        self.left_hand_pose = None
        self.right_hand_pose = None
        self.left_hand_inputs = None
        self.right_hand_inputs = None
        
        # 현재 그리퍼 상태 (MuJoCo 범위: -0.01 ~ 0.019)
        self.current_left_gripper = -0.005
        self.current_right_gripper = -0.005
        
        # Quest2ROS 좌표계 특성 기반 매핑 파라미터
        # 논문에서 확인된 정확도: 0.46mm, 주파수: 71.96Hz
        self.workspace_scale = {
            'x_forward': 2.0,    # 전진/후진 움직임
            'y_left': 4.0,       # 좌우 움직임 (베이스 회전용)
            'z_up': 2.5,         # 상하 움직임
            'rotation': 1.5      # 회전 움직임
        }
        
        # 베이스 회전 누적값 (연속적인 회전을 위해)
        self.left_base_rotation = 0.0
        self.right_base_rotation = 0.0
        self.prev_left_y = 0.0
        self.prev_right_y = 0.0
        
        # ROS Subscribers
        rospy.Subscriber('/q2r_left_hand_pose', PoseStamped, self.left_pose_callback)
        rospy.Subscriber('/q2r_right_hand_pose', PoseStamped, self.right_pose_callback)
        rospy.Subscriber('/q2r_left_hand_inputs', OVR2ROSInputs, self.left_inputs_callback)
        rospy.Subscriber('/q2r_right_hand_inputs', OVR2ROSInputs, self.right_inputs_callback)
        
        # ROS Publishers
        self.left_joint_pub = rospy.Publisher('/robot_arm_left_joint', Float64MultiArray, queue_size=1)
        self.right_joint_pub = rospy.Publisher('/robot_arm_right_joint', Float64MultiArray, queue_size=1)
        self.left_haptic_pub = rospy.Publisher('/robot_arm_left_haptic', Float64, queue_size=1)
        self.right_haptic_pub = rospy.Publisher('/robot_arm_right_haptic', Float64, queue_size=1)
        
        rospy.loginfo("Quest2ROS Optimized IK initialized")
        rospy.loginfo("Using Quest2ROS coordinate system: x(forward), y(left), z(up)")
    
    def left_pose_callback(self, msg):
        """왼손 포즈 콜백 - Quest2ROS 좌표계 기반"""
        self.left_hand_pose = msg.pose
        joint_values = self.quest_pose_to_joints(msg.pose, 'left')
        self.publish_joint_command(joint_values, 'left')
    
    def right_pose_callback(self, msg):
        """오른손 포즈 콜백 - Quest2ROS 좌표계 기반"""
        self.right_hand_pose = msg.pose
        joint_values = self.quest_pose_to_joints(msg.pose, 'right')
        self.publish_joint_command(joint_values, 'right')
    
    def left_inputs_callback(self, msg):
        """왼손 입력 콜백 - Quest2ROS 입력 구조 기반"""
        self.left_hand_inputs = msg
        
        # 그리퍼 제어 (press_index: 검지 누름 정도)
        gripper_range = 0.019 - (-0.01)  # MuJoCo 범위
        self.current_left_gripper = -0.01 + (msg.press_index * gripper_range)
        self.current_left_gripper = np.clip(self.current_left_gripper, -0.01, 0.019)
        
        # 햅틱 피드백 (button_upper: 상단 버튼)
        if msg.button_upper:
            haptic_msg = Float64()
            haptic_msg.data = 0.5
            self.left_haptic_pub.publish(haptic_msg)
    
    def right_inputs_callback(self, msg):
        """오른손 입력 콜백 - Quest2ROS 입력 구조 기반"""
        self.right_hand_inputs = msg
        
        # 그리퍼 제어
        gripper_range = 0.019 - (-0.01)
        self.current_right_gripper = -0.01 + (msg.press_index * gripper_range)
        self.current_right_gripper = np.clip(self.current_right_gripper, -0.01, 0.019)
        
        # 햅틱 피드백
        if msg.button_upper:
            haptic_msg = Float64()
            haptic_msg.data = 0.5
            self.right_haptic_pub.publish(haptic_msg)
    
    def quest_pose_to_joints(self, pose, side='left'):
        """
        Quest2ROS 좌표계를 로봇 조인트로 매핑
        Quest 좌표계: x(forward), y(left), z(up)
        """
        # Quest2ROS 위치 데이터 (미터 단위)
        x_forward = pose.position.x  # 앞/뒤
        y_left = pose.position.y     # 왼쪽/오른쪽
        z_up = pose.position.z       # 위/아래
        
        # Quest2ROS 방향 데이터 (쿼터니언 → 오일러)
        quat = [pose.orientation.x, pose.orientation.y,
                pose.orientation.z, pose.orientation.w]
        roll, pitch, yaw = tf_trans.euler_from_quaternion(quat)
        
        # 베이스 회전 계산 (연속적인 회전)
        if side == 'left':
            # Y 축 움직임 변화량으로 베이스 누적 회전
            y_delta = y_left - self.prev_left_y
            if abs(y_delta) < 0.5:  # 급격한 변화 무시 (노이즈 필터)
                self.left_base_rotation += y_delta * self.workspace_scale['y_left']
            self.prev_left_y = y_left
            
            base_rotation = self.left_base_rotation
            current_gripper = self.current_left_gripper
        else:
            # 오른손도 동일한 로직
            y_delta = y_left - self.prev_right_y  
            if abs(y_delta) < 0.5:
                self.right_base_rotation += y_delta * self.workspace_scale['y_left']
            self.prev_right_y = y_left
            
            base_rotation = self.right_base_rotation
            current_gripper = self.current_right_gripper
        
        # 조인트 매핑 (Quest2ROS 논문 기반)
        joint_values = [
            # Joint 1: Y축 움직임으로 베이스 회전 (전체 [-π,π] 범위 활용)
            np.clip(base_rotation, -3.142, 3.142),
            
            # Joint 2: Z축(상하) 움직임 → 어깨 관절
            np.clip(-z_up * self.workspace_scale['z_up'], -1.5, 1.5),
            
            # Joint 3: X축(전후) 움직임 → 팔꿈치 관절  
            np.clip(x_forward * self.workspace_scale['x_forward'], -1.5, 1.4),
            
            # Joint 4: 손목 회전 (pitch + yaw 조합)
            np.clip(
                (pitch * self.workspace_scale['rotation']) + 
                (yaw * self.workspace_scale['rotation'] * 0.5), 
                -1.7, 1.97
            ),
            
            # Gripper: 검지 누름 정도
            current_gripper
        ]
        
        return joint_values
    
    def publish_joint_command(self, joint_values, side):
        """조인트 명령 발행"""
        try:
            joint_cmd = Float64MultiArray()
            joint_cmd.data = joint_values
            
            if side == 'left':
                self.left_joint_pub.publish(joint_cmd)
                rospy.loginfo_throttle(2, f"Left Quest→Robot: {[round(x,3) for x in joint_values]}")
            else:
                self.right_joint_pub.publish(joint_cmd)
                rospy.loginfo_throttle(2, f"Right Quest→Robot: {[round(x,3) for x in joint_values]}")
                
        except Exception as e:
            rospy.logerr(f"Error publishing {side} joints: {e}")
    
    def reset_base_rotation(self):
        """베이스 회전 리셋 (필요시 호출)"""
        self.left_base_rotation = 0.0
        self.right_base_rotation = 0.0
        rospy.loginfo("Base rotations reset to zero")
    
    def run(self):
        """메인 루프"""
        rospy.loginfo("Quest2ROS Optimized IK running at 71.96Hz...")
        rospy.loginfo("Move VR controller LEFT/RIGHT for full [-π,π] base rotation")
        rospy.loginfo("Press Ctrl+C to stop")
        
        # 주기적으로 상태 정보 출력
        rate = rospy.Rate(1)  # 1Hz
        while not rospy.is_shutdown():
            rospy.loginfo_throttle(10, 
                f"Base rotations - Left: {self.left_base_rotation:.2f}, Right: {self.right_base_rotation:.2f}")
            rate.sleep()

if __name__ == "__main__":
    try:
        ik_solver = Quest2ROSOptimizedIK()
        ik_solver.run()
    except rospy.ROSInterruptException:
        rospy.loginfo("Quest2ROS IK solver stopped")
    except Exception as e:
        rospy.logerr(f"Error: {e}")
