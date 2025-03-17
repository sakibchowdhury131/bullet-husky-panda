import pyrealsense2 as rs
import numpy as np

# Start pipeline to access the camera
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust resolution if needed
pipeline.start(config)

# Get intrinsic parameters
profile = pipeline.get_active_profile()
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# Camera matrix (intrinsic parameters)
fx, fy = intrinsics.fx, intrinsics.fy  # Focal lengths
cx, cy = intrinsics.ppx, intrinsics.ppy  # Principal point

camera_matrix = np.array([
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1]
], dtype=np.float32)

print("Camera Matrix:\n", camera_matrix)

pipeline.stop()
