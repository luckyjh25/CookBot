import mujoco
import numpy as np
import time
from mujoco.glfw import glfw
import os

# Check current working directory (set to the directory containing scene.xml and omx.xml)
current_dir = os.getcwd()
print(f"Current working directory: {current_dir}")

# Load model
model_path = os.path.join(os.path.dirname(__file__), "scene.xml")
model = mujoco.MjModel.from_xml_path(model_path)
data = mujoco.MjData(model)

# Initialize GLFW and create window
glfw.init()
window = glfw.create_window(1200, 900, "OpenManipulator X Control", None, None)
glfw.make_context_current(window)
glfw.swap_interval(1)

# Set up MuJoCo visualization context
cam = mujoco.MjvCamera()
opt = mujoco.MjvOption()
scene = mujoco.MjvScene(model, maxgeom=10000)
context = mujoco.MjrContext(model, mujoco.mjtFontScale.mjFONTSCALE_150.value)

# Camera settings
cam.distance = 1.0  # Camera distance
cam.elevation = -20.0  # Camera elevation
cam.azimuth = 90.0  # Camera azimuth
cam.lookat = np.array([0.2, 0.0, 0.2])  # Camera viewpoint

# Mouse interaction variables
button_left = False
button_middle = False
button_right = False
lastx = 0
lasty = 0

# Control state variables
demo_mode = False    # Demo mode (False: keyboard control, True: automatic demo)
run_once = False     # Run demo only once
demo_step = 0        # Current demo step

# Global variables
joint_idx = 0  # Default selected joint

# Predefined poses
home_position = np.array([0.0, 0.0, 0.0, 0.0, 0.0])  # Home position
pick_position = np.array([1.57, 0.5, 0.5, -0.5, 0.0])  # Pick object position
place_position = np.array([-1.57, 0.5, 0.5, -0.5, 0.0])  # Place object position
gripper_open_val = -0.01  # Gripper open value
gripper_close_val = 0.019  # Gripper close value

# Keyboard/mouse callback functions
def keyboard(window, key, scancode, act, mods):
    global demo_mode, run_once, demo_step, joint_idx
    
    if act == glfw.PRESS:
        if key == glfw.KEY_ESCAPE:
            glfw.set_window_should_close(window, True)
        elif key == glfw.KEY_SPACE:
            # Spacebar: Toggle demo mode
            demo_mode = not demo_mode
            demo_step = 0
            print(f"Demo mode: {'Activated' if demo_mode else 'Deactivated'}")
        elif key == glfw.KEY_ENTER:
            # Enter: Run demo once
            run_once = True
            demo_step = 0
            print("Running demo once")
        elif key == glfw.KEY_H:
            # H: Move to home position
            set_joint_targets(home_position)
            print("Moving to home position")
        elif key == glfw.KEY_P:
            # P: Move to pick position
            set_joint_targets(pick_position)
            print("Moving to pick position")
        elif key == glfw.KEY_L:
            # L: Move to place position
            set_joint_targets(place_position)
            print("Moving to place position")
        elif key == glfw.KEY_O:
            # O: Open gripper
            current_pos = data.ctrl[:5].copy()
            current_pos[-1] = gripper_open_val
            set_joint_targets(current_pos)
            print("Opening gripper")
        elif key == glfw.KEY_C:
            # C: Close gripper
            current_pos = data.ctrl[:5].copy()
            current_pos[-1] = gripper_close_val
            set_joint_targets(current_pos)
            print("Closing gripper")
        elif key >= glfw.KEY_1 and key <= glfw.KEY_4:
            # Keys 1-4: Select joint
            joint_idx = key - glfw.KEY_1
            print(f"Joint {joint_idx+1} selected")
        elif key == glfw.KEY_UP:
            # Up arrow: Increase selected joint
            current_pos = data.ctrl[:5].copy()
            current_pos[joint_idx] += 0.1
            set_joint_targets(current_pos)
            print(f"Joint {joint_idx+1} value increased: {current_pos[joint_idx]:.2f}")
        elif key == glfw.KEY_DOWN:
            # Down arrow: Decrease selected joint
            current_pos = data.ctrl[:5].copy()
            current_pos[joint_idx] -= 0.1
            set_joint_targets(current_pos)
            print(f"Joint {joint_idx+1} value decreased: {current_pos[joint_idx]:.2f}")

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

# Function to set joint target values
def set_joint_targets(targets):
    # Joint control (position control)
    for i, target in enumerate(targets):
        if i < 5:  # 5 joints (4 main joints + gripper)
            data.ctrl[i] = target

# Simple joint interpolation (smoothly move from current position to target position)
def interpolate_joints(start_pos, end_pos, steps=100):
    for step in range(steps):
        alpha = step / steps
        current_pos = start_pos * (1 - alpha) + end_pos * alpha
        set_joint_targets(current_pos)
        
        # Advance simulation step
        mujoco.mj_step(model, data)
        
        # Render
        render_scene()
        
        # Wait for frame
        glfw.poll_events()
        if glfw.window_should_close(window):
            return False
    
    return True

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
        "SPACE: Toggle demo mode",
        "ENTER: Run demo once",
        "H: Home position",
        "P: Pick position",
        "L: Place position",
        "O: Open gripper",
        "C: Close gripper",
        "1-4: Select joint",
        "↑/↓: Control selected joint"
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

# Function to run demo step
def run_demo_step():
    global demo_step
    
    # Current joint position
    current_pos = data.ctrl[:5].copy()
    
    # Execute demo step by step
    if demo_step == 0:
        # 1. Move to home position
        print("Demo step 1: Moving to home position")
        if interpolate_joints(current_pos, home_position):
            demo_step += 1
            return True
    elif demo_step == 1:
        # 2. Open gripper
        print("Demo step 2: Opening gripper")
        current_pos[-1] = gripper_open_val
        if interpolate_joints(data.ctrl[:5], current_pos):
            demo_step += 1
            time.sleep(0.5)
            return True
    elif demo_step == 2:
        # 3. Move to pick position
        print("Demo step 3: Moving to pick position")
        if interpolate_joints(data.ctrl[:5], pick_position):
            demo_step += 1
            time.sleep(0.5)
            return True
    elif demo_step == 3:
        # 4. Close gripper (pick object)
        print("Demo step 4: Closing gripper")
        current_pos = data.ctrl[:5].copy()
        current_pos[-1] = gripper_close_val
        if interpolate_joints(data.ctrl[:5], current_pos):
            demo_step += 1
            time.sleep(0.5)
            return True
    elif demo_step == 4:
        # 5. Move to place position
        print("Demo step 5: Moving to place position")
        if interpolate_joints(data.ctrl[:5], place_position):
            demo_step += 1
            time.sleep(0.5)
            return True
    elif demo_step == 5:
        # 6. Open gripper (release object)
        print("Demo step 6: Opening gripper")
        current_pos = data.ctrl[:5].copy()
        current_pos[-1] = gripper_open_val
        if interpolate_joints(data.ctrl[:5], current_pos):
            demo_step = 0  # Demo complete, return to start
            time.sleep(0.5)
            return False  # One cycle complete
    
    return True  # Still in progress

# Main loop
try:
    # Initialize: Set to home position
    set_joint_targets(home_position)
    
    # Start guide message
    print("\n=== OpenManipulator X Control ===")
    print("ESC: Exit")
    print("SPACE: Toggle demo mode (continuous automatic demo)")
    print("ENTER: Run demo once")
    print("H: Home position")
    print("P: Pick position")
    print("L: Place position")
    print("O: Open gripper")
    print("C: Close gripper")
    print("1-4: Select joint")
    print("↑/↓: Control selected joint")
    
    while not glfw.window_should_close(window):
        # Advance simulation step
        mujoco.mj_step(model, data)
        
        # Check demo mode
        if demo_mode:
            # In demo mode, automatically run demo steps
            run_demo_step()
        elif run_once:
            # Run demo once mode
            if not run_demo_step():
                run_once = False  # Reset flag when demo is complete
        
        # Render scene
        render_scene()
        
        # Process events
        glfw.poll_events()

finally:
    # Clean up GLFW on exit
    glfw.terminate()