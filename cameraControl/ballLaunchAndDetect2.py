import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import time

# Configure serial connection (adjust COM port or ttyUSB as needed)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
time.sleep(1)  # Allow Arduino to initialize

# Rotation matrix and translation vector (camera to global transformation)
R = np.array([[-0.99418738,  0.08729485, -0.06301644],
              [ 0.06186601, -0.01582472, -0.997959  ],
              [-0.0881139 , -0.99605682,  0.01033216]])
t = np.array([0.85304391, 0.86452245, 1.01847464])

# Initialize RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()

# Enable depth and color streams
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

# Start streaming
pipeline.start(config)

# Get depth scale for converting depth values to meters
profile = pipeline.get_active_profile()
depth_sensor = profile.get_device().first_depth_sensor()
depth_scale = depth_sensor.get_depth_scale()

# Get camera intrinsics
intrinsics = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
fx, fy, cx, cy = intrinsics.fx, intrinsics.fy, intrinsics.ppx, intrinsics.ppy

# Function to convert depth to 3D camera coordinates
def depth_to_camera_coords(x, y, depth):
    z = depth * depth_scale
    if z == 0:
        return None
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    return np.array([X, Y, z])

# Function to transform camera coordinates to global coordinates
def camera_to_global_coords(camera_coords):
    return np.dot(R, camera_coords) + t

# Serial communication functions
def send_command(command):
    ser.write(command.encode())
    print(f"Sent: {command}")

def receive_response():
    while True:
        if ser.in_waiting > 0:
            response = ser.read().decode().strip()
            print(f"Received: {response}")
            return response

# Initialize previous frame for motion detection
previous_gray = None

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
            
            if radius > 10:  # Filter out small noise
                x, y, radius = int(x), int(y), int(radius)
                depth_value = depth_image[y, x]
                ball_camera_coords = depth_to_camera_coords(x, y, depth_value)

                if ball_camera_coords is not None:
                    ball_global_coords = camera_to_global_coords(ball_camera_coords)
                    print(f"Ball Global Position: {ball_global_coords}")
                    
                    cv2.circle(color_image, (x, y), radius, (0, 255, 0), 2)
                    cv2.putText(color_image, f"Global: {ball_global_coords[0]:.2f}, {ball_global_coords[1]:.2f}, {ball_global_coords[2]:.2f}",
                                (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        previous_gray = gray.copy()
        cv2.imshow("Color Image", color_image)
        cv2.imshow("Motion Mask", thresh)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('l'):
            send_command('1')
            while True:
                response = receive_response()
                if response == 'D':
                    print(f"Ball thrown at: {time.time()}")
                    send_command('0')
                    break
        if key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    ser.close()