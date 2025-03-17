import cv2
import numpy as np
import pyrealsense2 as rs

# Initialize variables
image_points = []
object_points = []
captured_image = None

# Define the camera intrinsic parameters
fx, fy, cx, cy = 609.4323, 609.5173, 315.48343, 247.94066
camera_matrix = np.array([
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1]
], dtype=np.float32)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)  # Assuming no lens distortion

# Callback function for mouse click event
def click_event(event, x, y, flags, param):
    global image_points
    if event == cv2.EVENT_LBUTTONDOWN:
        print(f"Clicked at: ({x}, {y})")
        image_points.append([x, y])

# Start the RealSense pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

# Capture a frame
frames = pipeline.wait_for_frames()
color_frame = frames.get_color_frame()
captured_image = np.asanyarray(color_frame.get_data())
pipeline.stop()

# Display image and collect 8 click points
cv2.imshow("Captured Image - Click 8 Points", captured_image)
cv2.setMouseCallback("Captured Image - Click 8 Points", click_event)

while len(image_points) < 8:
    cv2.waitKey(1)

cv2.destroyAllWindows()

# Convert image points to numpy array
image_points = np.array(image_points, dtype=np.float32)

# Manually enter corresponding 3D world coordinates
for i in range(8):
    x, y, z = map(float, input(f"Enter 3D coordinates (x, y, z) for point {i+1}: ").split())
    object_points.append([x, y, z])

object_points = np.array(object_points, dtype=np.float32)

# Compute Rotation and Translation using SolvePnP
success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

if success:
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)
    print("\nRotation Matrix (R):\n", R)
    print("Translation Vector (t):\n", t)
else:
    print("SolvePnP failed to compute the transformation.")
