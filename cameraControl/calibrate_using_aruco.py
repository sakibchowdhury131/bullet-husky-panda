import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

# ------------------------------
# 1. Setup the ArUco GridBoard for Calibration
# ------------------------------
# Choose an ArUco dictionary, e.g., DICT_6X6_250.
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
marker_length = 0.04      # Marker side length in meters
marker_separation = 0.01  # Separation between markers in meters

# Create a GridBoard with a known layout.
board = aruco.GridBoard_create(
    markersX=5, 
    markersY=7, 
    markerLength=marker_length, 
    markerSeparation=marker_separation, 
    dictionary=aruco_dict
)

# ------------------------------
# 2. Prepare to Collect Calibration Data
# ------------------------------
all_corners = []      # To store detected marker corners for each captured frame
all_ids = []          # To store corresponding marker IDs for each frame
marker_counter = []   # To store the number of detected markers per frame

print("Press 'c' to capture a frame for calibration. Press 'q' to quit capturing.")

# ------------------------------
# 3. Setup Intel RealSense Pipeline
# ------------------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        # Wait for a new set of frames from the RealSense camera
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue
        
        # Convert RealSense color frame to a NumPy array
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers in the grayscale image
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
        
        # Draw detected markers for visualization
        if ids is not None:
            frame_markers = aruco.drawDetectedMarkers(frame.copy(), corners, ids)
        else:
            frame_markers = frame
        
        cv2.imshow("Calibration", frame_markers)
        key = cv2.waitKey(1)
        
        # Capture the frame if the user presses 'c' and markers are detected
        if key == ord('c'):
            if ids is not None and len(ids) > 0:
                all_corners.append(corners)
                all_ids.append(ids)
                marker_counter.append(len(ids))
                print(f"Captured frame with {len(ids)} markers.")
            else:
                print("No markers detected in this frame; try again.")
        elif key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

# ------------------------------
# 4. Perform Camera Calibration Using Captured Data
# ------------------------------
if len(all_corners) > 0:
    # Use the size of the last captured grayscale image as the image size (width, height)
    image_size = gray.shape[::-1]
    
    # Calibrate using the ArUco board data
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = aruco.calibrateCameraAruco(
        all_corners, all_ids, marker_counter, board, image_size, None, None
    )
    
    print("\nCalibration complete.")
    print("Reprojection error:", ret)
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", dist_coeffs)
    
    # Optionally, print extrinsic parameters (rotation and translation) for each calibration frame
    for i, (rvec, tvec) in enumerate(zip(rvecs, tvecs)):
        R, _ = cv2.Rodrigues(rvec)
        print(f"\nFrame {i+1} Rotation Matrix:\n{R}")
        print(f"Frame {i+1} Translation Vector:\n{tvec.reshape(3)}")
else:
    print("No calibration frames were captured.")
