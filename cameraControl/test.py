import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import time

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

# Function to convert depth to 3D world coordinates
def depth_to_3d(x, y, depth):
    z = depth * depth_scale
    if z == 0:
        return None
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    return np.array([X, Y, z])

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
                ball_position = depth_to_3d(x, y, depth_value)
                
                if ball_position is not None:
                    current_time = time.time()
                    
                    if ball_launched and not ball_detected_after_launch:
                        print(f"Ball detected at {current_time - launch_time:.3f} seconds after launch: {ball_position}")
                        ball_detected_after_launch = True
                    
                    cv2.circle(color_image, (x, y), radius, (0, 255, 0), 2)
                    cv2.putText(color_image, f"3D: {ball_position[0]:.2f}, {ball_position[1]:.2f}, {ball_position[2]:.2f}", 
                                (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
        
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
                ball_detected_after_launch = False
                send_command('0')
finally:
    pipeline.stop()
    ser.close()
    cv2.destroyAllWindows()
