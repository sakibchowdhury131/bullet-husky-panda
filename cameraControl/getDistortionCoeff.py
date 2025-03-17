import pyrealsense2 as rs
import numpy as np

# Start the pipeline
pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # Adjust resolution if needed
pipeline.start(config)

# Get the camera intrinsics
profile = pipeline.get_active_profile()
intrinsics = profile.get_stream(rs.stream.color).as_video_stream_profile().get_intrinsics()

# Extract distortion coefficients
distortion_model = intrinsics.model
dist_coeffs = np.array(intrinsics.coeffs)  # [k1, k2, p1, p2, k3]

# Print results
print("Distortion Model:", distortion_model)
print("Distortion Coefficients:", dist_coeffs)

pipeline.stop()