import pyrealsense2 as rs
import numpy as np
import cv2

# Create a context object. This manages all connected RealSense devices
context = rs.context()
devices = context.query_devices()

if len(devices) < 2:
    print("Error: Less than 2 RealSense cameras found!")
    exit()

# Initialize pipelines for each camera
pipelines = []
serial_numbers = []

for device in devices:
    serial_number = device.get_info(rs.camera_info.serial_number)
    serial_numbers.append(serial_number)
    pipeline = rs.pipeline()
    
    # Configure streams
    config = rs.config()
    config.enable_device(serial_number)  # Select specific camera
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)  # RGB stream
    
    # Start streaming
    pipeline.start(config)
    pipelines.append(pipeline)

try:
    while True:
        frames = [pipeline.wait_for_frames() for pipeline in pipelines]
        color_frames = [frame.get_color_frame() for frame in frames]
        
        # Convert to numpy arrays
        color_images = [np.asanyarray(frame.get_data()) for frame in color_frames]
        
        # Display the images
        for i, img in enumerate(color_images):
            cv2.imshow(f"Camera {serial_numbers[i]}", img)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

finally:
    # Stop all pipelines
    for pipeline in pipelines:
        pipeline.stop()
    cv2.destroyAllWindows()
