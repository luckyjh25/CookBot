import numpy as np
import mujoco
import time
import os
import threading
from mujoco.glfw import glfw

# ROS2 관련 import 추가
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32

# MuJoCoControlNode 클래스 - ROS2와 MuJoCo 연결
class MuJoCoControlNode(Node):
    def __init__(self, model, data, left_joints, right_joints):
        super().__init__('mujoco_control_node')
        
        # MuJoCo 모델과 데이터 저장
        self.model = model
        self.data = data
        self.left_joints = left_joints
        self.right_joints = right_joints
        
        # 조인트 값 저장 변수
        self.left_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.right_joint_values = [0.0, 0.0, 0.0, 0.0, 0.0]
        
        # ROS2 토픽 구독
        self.left_joints_sub = self.create_subscription(
            JointState, '/left_arm_joints', self.left_joints_callback, 10)
        self.right_joints_sub = self.create_subscription(
            JointState, '/right_arm_joints', self.right_joints_callback, 10)
        
        # 햅틱 피드백 발행자
        self.left_haptic_pub = self.create_publisher(Float32, '/left_hand_haptic_feedback', 10)
        self.right_haptic_pub = self.create_publisher(Float32, '/right_hand_haptic_feedback', 10)
        
        self.get_logger().info('MuJoCo Control Node started')
    
    def left_joints_callback(self, msg):
        """왼쪽 팔 조인트 상태 콜백"""
        for i, value in enumerate(msg.position):
            if i < len(self.left_joint_values):
                self.left_joint_values[i] = value
        
        # 디버깅을 위한 로깅
        self.get_logger().info(f'Left joints received: {self.left_joint_values}')
    
    def right_joints_callback(self, msg):
        """오른쪽 팔 조인트 상태 콜백"""
        for i, value in enumerate(msg.position):
            if i < len(self.right_joint_values):
                self.right_joint_values[i] = value
        
        # 디버깅을 위한 로깅
        self.get_logger().info(f'Right joints received: {self.right_joint_values}')
    
    def update_robot(self):
        """컨트롤러 데이터를 기반으로 로봇 업데이트"""
        # ROS2에서 받은 조인트 순서: [base_rotation, shoulder, elbow, wrist, gripper]
        # MuJoCo actuator 순서와 매핑
        
        # 왼쪽 팔 조인트 적용
        if len(self.left_joint_values) >= 5:
            self.data.ctrl[self.left_joints[0]] = self.left_joint_values[0]  # 베이스 회전 (가장 중요!)
            self.data.ctrl[self.left_joints[1]] = self.left_joint_values[1]  # 어깨
            self.data.ctrl[self.left_joints[2]] = self.left_joint_values[2]  # 팔꿈치
            self.data.ctrl[self.left_joints[3]] = self.left_joint_values[3]  # 손목
            self.data.ctrl[self.left_joints[4]] = self.left_joint_values[4]  # 그리퍼
        
        # 오른쪽 팔 조인트 적용
        if len(self.right_joint_values) >= 5:
            self.data.ctrl[self.right_joints[0]] = self.right_joint_values[0]  # 베이스 회전
            self.data.ctrl[self.right_joints[1]] = self.right_joint_values[1]  # 어깨
            self.data.ctrl[self.right_joints[2]] = self.right_joint_values[2]  # 팔꿈치
            self.data.ctrl[self.right_joints[3]] = self.right_joint_values[3]  # 손목
            self.data.ctrl[self.right_joints[4]] = self.right_joint_values[4]  # 그리퍼
        
        # 디버깅: 실제 적용된 제어 값 출력
        self.get_logger().debug(f'Left ctrl values: {[self.data.ctrl[j] for j in self.left_joints]}')
        self.get_logger().debug(f'Right ctrl values: {[self.data.ctrl[j] for j in self.right_joints]}')
    
    def send_haptic_feedback(self, side, value):
        """햅틱 피드백 전송"""
        msg = Float32()
        msg.data = value
        
        if side == 'left':
            self.left_haptic_pub.publish(msg)
        elif side == 'right':
            self.right_haptic_pub.publish(msg)

# Print current directory for debugging
print(f"Current working directory: {os.getcwd()}")

try:
    # Load the model from the existing XML file
    print("Attempting to load the model...")
    model = mujoco.MjModel.from_xml_path('/home/joonghyun/CookingBot/Mujoco/dual_arm_robot.xml')
    data = mujoco.MjData(model)
    print("Model loaded successfully!")
    
    # 디버깅: 모델 정보 출력
    print(f"Number of actuators: {model.nu}")
    print("Actuator names:")
    for i in range(model.nu):
        actuator_name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_ACTUATOR, i)
        print(f"  [{i}]: {actuator_name}")
    
    # Initialize GLFW and create window
    print("Initializing GLFW...")
    glfw.init()
    window = glfw.create_window(1200, 900, "Dual Robot Box Lifting", None, None)
    glfw.make_context_current(window)
    glfw.swap_interval(1)
    
    # Set up MuJoCo visualization context
    cam = mujoco.MjvCamera()
    opt = mujoco.MjvOption()
    scene = mujoco.MjvScene(model, maxgeom=10000)
    context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

    # Camera settings - positioned to view the facing arms and box
    cam.distance = 1.2  # Camera distance
    cam.elevation = -20.0  # Camera elevation angle
    cam.azimuth = 0.0  # Camera azimuth - looking from the side
    cam.lookat = np.array([0.0, 0.0, 0.3])  # Camera viewpoint (center between the robots)

    # Mouse interaction variables
    button_left = False
    button_middle = False
    button_right = False
    lastx = 0
    lasty = 0

    # Keyboard/mouse callback functions
    def keyboard(window, key, scancode, act, mods):
        if act == glfw.PRESS:
            if key == glfw.KEY_ESCAPE:
                glfw.set_window_should_close(window, True)
            elif key == glfw.KEY_R:
                # Reset position
                reset_position()
                print("Reset to initial position")
            elif key == glfw.KEY_SPACE:
                # Start box lifting task
                box_lifting_task()
                print("Starting box lifting task")
            elif key == glfw.KEY_H:
                # Ready position
                ready_position()
                print("Moving to ready position")
            elif key == glfw.KEY_T:
                # Toggle ROS2 control
                global use_ros2_control
                use_ros2_control = not use_ros2_control
                print(f"ROS2 control: {'ON' if use_ros2_control else 'OFF'}")
            elif key == glfw.KEY_D:
                # 디버깅: 현재 조인트 상태 출력
                print("=== Current Joint States ===")
                print(f"Left joints: {[data.ctrl[j] for j in left_joints]}")
                print(f"Right joints: {[data.ctrl[j] for j in right_joints]}")
                print(f"Left positions: {[data.qpos[j] for j in range(5)]}")
                print(f"Right positions: {[data.qpos[j] for j in range(5, 10)]}")

    def mouse_button(window, button, act, mods):
        global button_left, button_middle, button_right
        
        if button == glfw.MOUSE_BUTTON_LEFT:
            button_left = (act == glfw.PRESS)
        elif button == glfw.MOUSE_BUTTON_MIDDLE:
            button_middle = (act == glfw.PRESS)
        elif button == glfw.MOUSE_BUTTON_RIGHT:
            button_right = (act == glfw.PRESS)
        
        # Update mouse position
        global lastx, lasty
        lastx, lasty = glfw.get_cursor_pos(window)

    def mouse_move(window, xpos, ypos):
        global lastx, lasty, button_left, button_middle, button_right
        
        # Only move camera when mouse button is pressed
        if button_left:
            # Camera rotation
            dy = 0.01 * (ypos - lasty)
            dx = 0.01 * (xpos - lastx)
            cam.elevation = np.clip(cam.elevation - dy*100, -90, 90)
            cam.azimuth = (cam.azimuth + dx*100) % 360
        elif button_middle:
            # Camera panning
            dx = 0.001 * (xpos - lastx)
            dy = 0.001 * (ypos - lasty)
            cam.lookat[0] += -dx*cam.distance
            cam.lookat[1] += dy*cam.distance
        elif button_right:
            # Camera zoom
            dy = 0.01 * (ypos - lasty)
            cam.distance = np.clip(cam.distance + dy, 0.1, 5.0)
        
        lastx = xpos
        lasty = ypos

    def scroll(window, xoffset, yoffset):
        # Zoom in/out with scroll
        cam.distance = np.clip(cam.distance - 0.1 * yoffset, 0.1, 5.0)

    # Register callbacks
    glfw.set_key_callback(window, keyboard)
    glfw.set_mouse_button_callback(window, mouse_button)
    glfw.set_cursor_pos_callback(window, mouse_move)
    glfw.set_scroll_callback(window, scroll)

    # Scene rendering function
    def render_scene():
        viewport_width, viewport_height = glfw.get_framebuffer_size(window)
        viewport = mujoco.MjrRect(0, 0, viewport_width, viewport_height)
        
        # Update scene
        mujoco.mjv_updateScene(
            model, data, opt, None, cam, 
            mujoco.mjtCatBit.mjCAT_ALL.value, scene
        )
        
        # Render
        mujoco.mjr_render(viewport, scene, context)
        
        # Text overlay (key guide)
        text = [
            "ESC: Exit",
            "SPACE: Start box lifting task",
            "R: Reset position",
            "H: Ready position",
            f"T: Toggle ROS2 control (currently {'ON' if use_ros2_control else 'OFF'})",
            "D: Debug joint states"
        ]
        overlay = "\n".join(text)
        mujoco.mjr_overlay(
            mujoco.mjtFont.mjFONT_NORMAL, 
            mujoco.mjtGridPos.mjGRID_TOPLEFT, 
            viewport, 
            overlay, 
            "", 
            context
        )
        
        # Swap buffers
        glfw.swap_buffers(window)

    # Get actuator indices for both robots
    left_joints = [
        model.actuator('left_actuator_joint1').id,      # 베이스 회전
        model.actuator('left_actuator_joint2').id,      # 어깨
        model.actuator('left_actuator_joint3').id,      # 팔꿈치
        model.actuator('left_actuator_joint4').id,      # 손목
        model.actuator('left_actuator_gripper_joint').id # 그리퍼
    ]

    right_joints = [
        model.actuator('right_actuator_joint1').id,      # 베이스 회전
        model.actuator('right_actuator_joint2').id,      # 어깨
        model.actuator('right_actuator_joint3').id,      # 팔꿈치
        model.actuator('right_actuator_joint4').id,      # 손목
        model.actuator('right_actuator_gripper_joint').id # 그리퍼
    ]

    print(f"Left joint actuator IDs: {left_joints}")
    print(f"Right joint actuator IDs: {right_joints}")

    # Reset to initial pose - both arms in position to reach the box
    def reset_position():
        # Left arm initial position (facing right)
        data.ctrl[left_joints[0]] = 0.0       # Base joint
        data.ctrl[left_joints[1]] = -0.3      # Shoulder joint slightly down
        data.ctrl[left_joints[2]] = 0.6       # Elbow joint forward
        data.ctrl[left_joints[3]] = 0.0       # Wrist joint straight
        data.ctrl[left_joints[4]] = 0.01      # Gripper open

        # Right arm initial position (facing left)
        data.ctrl[right_joints[0]] = 0.0      # Base joint
        data.ctrl[right_joints[1]] = -0.3     # Shoulder joint slightly down
        data.ctrl[right_joints[2]] = 0.6      # Elbow joint forward
        data.ctrl[right_joints[3]] = 0.0      # Wrist joint straight
        data.ctrl[right_joints[4]] = 0.01     # Gripper open

    # Move to ready position - arms positioned to reach for the box
    def ready_position():
        # Left arm ready position (reaching toward center)
        data.ctrl[left_joints[0]] = 0.0       # Base joint 
        data.ctrl[left_joints[1]] = -0.4      # Shoulder joint down
        data.ctrl[left_joints[2]] = 0.8       # Elbow joint forward
        data.ctrl[left_joints[3]] = 0.2       # Wrist joint tilted for grip
        data.ctrl[left_joints[4]] = 0.015     # Gripper open
        
        # Right arm ready position (reaching toward center)
        data.ctrl[right_joints[0]] = 0.0      # Base joint
        data.ctrl[right_joints[1]] = -0.4     # Shoulder joint down
        data.ctrl[right_joints[2]] = 0.8      # Elbow joint forward
        data.ctrl[right_joints[3]] = 0.2      # Wrist joint tilted for grip
        data.ctrl[right_joints[4]] = 0.015    # Gripper open

    # Box manipulation task: Both robots pick up and hand off the box
    def box_lifting_task():
        print("Starting box manipulation task...")
        
        # 베이스 회전 테스트 추가
        print("Testing base rotation...")
        data.ctrl[left_joints[0]] = 0.5   # 왼쪽 베이스를 30도 회전
        data.ctrl[right_joints[0]] = -0.5  # 오른쪽 베이스를 -30도 회전
        simulate(2.0)
        
        # 원래 위치로 복귀
        data.ctrl[left_joints[0]] = 0.0
        data.ctrl[right_joints[0]] = 0.0
        simulate(1.0)
        
        # 나머지 기존 태스크 계속...
        # 1. Move both arms to box position
        # Left arm moves to box from left side
        data.ctrl[left_joints[0]] = 0.0        # Base joint
        data.ctrl[left_joints[1]] = -0.6       # Shoulder down
        data.ctrl[left_joints[2]] = 1.0        # Elbow bent
        data.ctrl[left_joints[3]] = 0.3        # Wrist tilted
        data.ctrl[left_joints[4]] = 0.015      # Gripper open
        
        # Right arm moves to box from right side
        data.ctrl[right_joints[0]] = 0.0       # Base joint
        data.ctrl[right_joints[1]] = -0.6      # Shoulder down
        data.ctrl[right_joints[2]] = 1.0       # Elbow bent
        data.ctrl[right_joints[3]] = 0.3       # Wrist tilted
        data.ctrl[right_joints[4]] = 0.015     # Gripper open
        
        simulate(1.5)
        
        # 2. Close grippers to hold box from both sides
        data.ctrl[left_joints[4]] = -0.005     # Close left gripper
        data.ctrl[right_joints[4]] = -0.005    # Close right gripper
        simulate(1.0)
        
        # 3. Lift box together
        for i in range(15):
            # Gradually lift both arms
            data.ctrl[left_joints[1]] = -0.6 + i * 0.04   # Lift shoulder joint
            data.ctrl[right_joints[1]] = -0.6 + i * 0.04  # Lift shoulder joint
            simulate(0.1)
        
        # 4. Hold at highest position
        simulate(1.5)
        
        # 5. Pass box from left to right (left loosens, right holds)
        data.ctrl[left_joints[4]] = 0.015     # Open left gripper
        simulate(1.0)
        
        # 6. Left arm moves away
        data.ctrl[left_joints[2]] = 0.5       # Left arm pulls back
        data.ctrl[left_joints[1]] = -0.3      # Left arm moves up
        simulate(1.0)
        
        # 7. Right arm lowers box
        for i in range(15):
            # Gradually lower right arm
            data.ctrl[right_joints[1]] = -0.0 - i * 0.04  # Lower shoulder joint
            simulate(0.1)
        
        # 8. Place box down and release
        data.ctrl[right_joints[4]] = 0.015    # Open right gripper
        simulate(1.0)
        
        # 9. Return both arms to original positions
        data.ctrl[left_joints[0]] = 0.0       # Reset left base joint
        data.ctrl[left_joints[1]] = -0.3      # Reset left shoulder
        data.ctrl[left_joints[2]] = 0.6       # Reset left elbow
        data.ctrl[left_joints[3]] = 0.0       # Reset left wrist
        
        data.ctrl[right_joints[0]] = 0.0      # Reset right base joint  
        data.ctrl[right_joints[1]] = -0.3     # Reset right shoulder
        data.ctrl[right_joints[2]] = 0.6      # Reset right elbow  
        data.ctrl[right_joints[3]] = 0.0      # Reset right wrist
        simulate(2.0)
        
        print("Box manipulation task completed")

    # Simulate for specified duration with visualization
    def simulate(duration):
        start_time = time.time()
        while time.time() - start_time < duration:
            mujoco.mj_step(model, data)
            render_scene()
            glfw.poll_events()
            if glfw.window_should_close(window):
                return False
            time.sleep(0.01)
        return True

    # ROS2 초기화
    print("Initializing ROS2...")
    rclpy.init()
    ros_node = MuJoCoControlNode(model, data, left_joints, right_joints)

    # ROS2 스핀 스레드 시작
    def ros_spin():
        while not glfw.window_should_close(window):
            rclpy.spin_once(ros_node, timeout_sec=0.001)
            time.sleep(0.001)

    ros_thread = threading.Thread(target=ros_spin)
    ros_thread.daemon = True
    ros_thread.start()

    # 기본적으로 ROS2 제어 사용
    use_ros2_control = True

    # Main loop
    try:
        print("=== Dual Robot Box Lifting Control ===")
        print("ESC: Exit")
        print("SPACE: Start box lifting task")
        print("R: Reset position")
        print("H: Ready position")
        print("T: Toggle ROS2 control")
        print("D: Debug joint states")
        
        print("Initializing robots...")
        reset_position()
        
        while not glfw.window_should_close(window):
            # ROS2 제어가 활성화되어 있으면 ROS2에서 받은 조인트 값으로 로봇 업데이트
            if use_ros2_control:
                ros_node.update_robot()
            
            # Advance simulation step
            mujoco.mj_step(model, data)
            
            # Render scene
            render_scene()
            
            # Process events
            glfw.poll_events()
            
            # Delay to control simulation speed
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("Simulation terminated by user")
    finally:
        # Clean up ROS2 on exit
        if 'ros_node' in locals():
            ros_node.destroy_node()
        rclpy.shutdown()
        
        # Clean up GLFW on exit
        glfw.terminate()

except Exception as e:
    print(f"Error: {e}")
    print("\nTroubleshooting steps:")
    print("1. Make sure dual_arm_robot.xml is in the current directory")
    print("2. Check that all mesh files referenced in the XML are in the assets directory")
    print("3. Try running the script again")
