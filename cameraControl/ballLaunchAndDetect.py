import pyrealsense2 as rs
import numpy as np
import cv2
import serial
import time

# Configure serial connection (adjust COM port or ttyUSB as needed)
ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)  # Use '/dev/ttyUSB0' for Linux
time.sleep(1)  # Give time for the Arduino to initialize

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

# Function to convert depth to 3D world coordinates
def depth_to_3d(x, y, depth):
    z = depth * depth_scale
    if z == 0:  # Ignore invalid depth points
        return None
    X = (x - cx) * z / fx
    Y = (y - cy) * z / fy
    return np.array([X, Y, z])

# Serial communication functions
def send_command(command):
    """Send a character command to Arduino."""
    ser.write(command.encode())  # Send character
    print(f"Sent: {command}")

def receive_response():
    """Wait for a single-character response from Arduino."""
    while True:
        if ser.in_waiting > 0:
            response = ser.read().decode().strip()
            print(f"Received: {response}")
            return response

# Initialize previous frame for motion detection
previous_gray = None

try:
    while True:
        # Capture frames
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        # Convert to numpy arrays
        depth_image = np.asanyarray(depth_frame.get_data())
        color_image = np.asanyarray(color_frame.get_data())

        # Convert to grayscale
        gray = cv2.cvtColor(color_image, cv2.COLOR_BGR2GRAY)

        # If first frame, store and continue
        if previous_gray is None:
            previous_gray = gray
            continue

        # Compute absolute difference between current and previous frame
        frame_diff = cv2.absdiff(previous_gray, gray)

        # Apply threshold to highlight differences
        _, thresh = cv2.threshold(frame_diff, 30, 255, cv2.THRESH_BINARY)

        # Find contours of the moving regions
        contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            (x, y), radius = cv2.minEnclosingCircle(largest_contour)
            
            if radius > 10:  # Filter out small noise
                x, y, radius = int(x), int(y), int(radius)

                # Get depth value at detected position
                depth_value = depth_image[y, x]
                ball_position = depth_to_3d(x, y, depth_value)

                if ball_position is not None:
                    print(f"Moving object detected at: {ball_position}")

                    # Draw detection circle
                    cv2.circle(color_image, (x, y), radius, (0, 255, 0), 2)
                    cv2.putText(color_image, f"3D: {ball_position[0]:.2f}, {ball_position[1]:.2f}, {ball_position[2]:.2f}", 
                                (x + 10, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        # Update previous frame
        previous_gray = gray.copy()

        # Display results
        cv2.imshow("Color Image", color_image)
        cv2.imshow("Motion Mask", thresh)

        # Check for user input to launch the ball
        key = cv2.waitKey(1) & 0xFF
        if key == ord('l'):
            send_command('1')
            while True:
                response = receive_response()
                if response == 'D':
                    print(f"Ball thrown at: {time.time()}")
                    send_command('0')
                    break

        # Exit on 'q' key
        if key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()
    ser.close()  # Close the serial connection
