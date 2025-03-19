import pyrealsense2 as rs
import numpy as np
import cv2
import cv2.aruco as aruco

# ------------------------------
# 1. Setup the ArUco GridBoard for Calibration
# ------------------------------
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)
marker_length = 0.04      # Marker side length in meters
marker_separation = 0.01  # Separation between markers in meters
board = aruco.GridBoard_create(
    markersX=5,
    markersY=7,
    markerLength=marker_length,
    markerSeparation=marker_separation,
    dictionary=aruco_dict
)

# ------------------------------
# 2. Prepare Containers for Calibration Data
# ------------------------------
all_corners = []      # List to store detected marker corners for each frame
all_ids = []          # List to store corresponding marker IDs for each frame
marker_counter = []   # Number of markers detected per frame

print("Press 'c' to capture a frame for calibration. Press 'q' to finish capturing.")

# ------------------------------
# 3. Setup Intel RealSense Pipeline for Color Capture
# ------------------------------
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
pipeline.start(config)

try:
    while True:
        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        if not color_frame:
            continue

        # Convert the RealSense color frame to a NumPy array and then to grayscale
        frame = np.asanyarray(color_frame.get_data())
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect ArUco markers in the grayscale image
        corners, ids, _ = aruco.detectMarkers(gray, aruco_dict)
        
        if ids is not None:
            processed_corners = []
            unprocessed_corners = []
            for corner in corners:
                # Check the shape:
                # If shape is (1,4,2), transpose to (4,1,2)
                if corner.shape[0] == 1 and corner.shape[1] == 4:
                    corner = corner.transpose(1, 0, 2)
                # Now each corner should be of shape (4,1,2)
                processed_corners.append(corner.astype(np.float32))
            
            # Reshape IDs to a column vector and convert to int32
            processed_ids = ids.reshape(-1, 1).astype(np.int32)
            frame_markers = aruco.drawDetectedMarkers(frame.copy(), processed_corners, processed_ids)
        else:
            processed_corners = []
            processed_ids = None
            frame_markers = frame
        
        cv2.imshow("Calibration", frame_markers)
        key = cv2.waitKey(1)
        
        # Capture frame when 'c' is pressed (if markers are detected)
        if key == ord('c'):
            if processed_ids is not None and len(processed_ids) > 0:
                all_corners.append(processed_corners)
                all_ids.append(processed_ids)
                marker_counter.append(len(processed_ids))
                print(f"Captured frame with {len(processed_ids)} markers.")
            else:
                print("No markers detected in this frame.")
        elif key == ord('q'):
            break

finally:
    pipeline.stop()
    cv2.destroyAllWindows()

# ------------------------------
# 4. Perform Calibration and Extract Intrinsic Parameters
# ------------------------------
if len(all_corners) > 0:
    # Use the size of the last captured grayscale image as (width, height)
    image_size = gray.shape[::-1]
    
    # Convert marker_counter to a NumPy array of type int32
    marker_counter = np.array(marker_counter, dtype=np.int32)
    # Calibrate using the ArUco board data.
    ret, camera_matrix, dist_coeffs, _, _ = aruco.calibrateCameraAruco(
        np.array(all_corners), np.array(all_ids), marker_counter, board, image_size, None, None
    )
    
    print("\nCalibration complete.")
    print("Reprojection error:", ret)
    print("Camera Matrix:\n", camera_matrix)
    print("Distortion Coefficients:\n", dist_coeffs)
else:
    print("No calibration frames were captured.")
