import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import time

ball_init_pos = np.array([1.25, 0, 0.8362])
# Define the transformation parameters
R = np.array([[-0.99297062,  0.08525993, -0.08209811],
 [ 0.08062718, -0.02056058, -0.99653225],
 [-0.08665226, -0.99614658,  0.01354179]]
)

t = np.array([0.93206617, 0.82395536, 0.89772704])

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get depth scale
profile = pipeline.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Get camera intrinsics
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy

# Setup serial communication
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(1)

# Function to convert depth to 3D world coordinates (camera frame)
def depth_to_3d(x, y, depth):
    z = depth * depth_scale
    if z == 0:
        return None
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    return np.array([X, Y, z])

# Function to transform camera coordinates to global coordinates
def transform_to_global(camera_coords):
    return R @ camera_coords + t

# Function to send command to ball launcher
def send_command(command):
    ser.write(command.encode())
    print(f"Sent: {command}")

def receive_response():
    while True:
        if ser.in_waiting > 0:
            response = ser.read().decode().strip()
            print(f"Received: {response}")
            return response

previous_gray = None
ball_launched = False
launch_time = None
ball_detected_after_launch = False

# Persist only the position that was printed after launch
persisted_ball_pos = None
persisted_ball_screen_pos = None  # (x, y) in camera frame

try:
    while True:
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        
        if not depth_frame or not color_frame:
            continue
        
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())
        
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)
        
        if previous_gray is None:
            previous_gray = gray
            continue

        frame_diff = cv2.absdiff(previous_gray, gray)
        _, thresh = cv2.threshold(frame_diff, 30, 255, cv2.THRESH_BINARY)
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            if radius > 10:
                x, y, radius = int(x), int(y), int(radius)
                depth_value = depth_image[y, x]
                ball_position_cam = depth_to_3d(x, y, depth_value)
                
                if ball_position_cam is not None:
                    ball_position_global = transform_to_global(ball_position_cam)

                    # Check if the x-coordinate in global frame is within the valid range
                    if 0 <= ball_position_global[0] <= 1.25:
                        # Capture and persist position **only** on first detection after launch
                        if ball_launched and not ball_detected_after_launch:
                            current_time = time.time()
                            elapsed_time = current_time - launch_time
                            vel = (ball_position_global - ball_init_pos) / elapsed_time
                            print(f"Ball detected at {elapsed_time:.3f} sec after launch: {ball_position_global}")
                            print(f"Ball estimated Velocity: {vel} ms^-1")
                            # Store **this exact position** for future display
                            persisted_ball_pos = ball_position_global
                            persisted_ball_screen_pos = (x, y)
                            ball_detected_after_launch = True  # Prevent overwriting

        # Display the persisted position (ONLY the first valid detection after launch)
        if persisted_ball_screen_pos is not None:
            cv2.circle(color_image, persisted_ball_screen_pos, 10, (0, 255, 0), 2)
            cv2.putText(color_image, "Persisted Ball Position", 
                        (persisted_ball_screen_pos[0] + 10, persisted_ball_screen_pos[1] - 10), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            # Display persisted global coordinates
            if persisted_ball_pos is not None:
                cv2.putText(color_image, f"Global: {persisted_ball_pos[0]:.2f}, {persisted_ball_pos[1]:.2f}, {persisted_ball_pos[2]:.2f}", 
                            (persisted_ball_screen_pos[0] + 10, persisted_ball_screen_pos[1] + 10), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 2)

        previous_gray = gray.copy()

        cv2.imshow("Color Image", color_image)
        cv2.imshow("Motion Mask", thresh)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            break
        elif key == ord('l') and not ball_launched:
            send_command('1')
            response = receive_response()
            if response == 'D':
                launch_time = time.time()
                print(f"Ball thrown at: {launch_time}")
                ball_launched = True
                ball_detected_after_launch = False  # Reset for next detection
                persisted_ball_pos = None  # Clear previous persisted position
                persisted_ball_screen_pos = None  # Clear previous position on screen
                send_command('0')
finally:
    pipeline.stop()
    ser.close()
    cv2.destroyAllWindows()
