import cv2
import cv2.aruco as aruco
import numpy as np
import os

# ------------------------------
# 1. Choose an ArUco Dictionary
# ------------------------------
# For example, using the 6x6 dictionary with 250 markers.
aruco_dict = aruco.Dictionary_get(aruco.DICT_6X6_250)

# Define the marker size (in pixels)
marker_size = 200  # You can change this value for larger or smaller markers

# ------------------------------
# 2. Create an Output Directory for Saving Markers
# ------------------------------
output_dir = "aruco_markers"
if not os.path.exists(output_dir):
    os.makedirs(output_dir)

# ------------------------------
# 3. Generate, Display, and Save Markers
# ------------------------------
# Generate markers for a range of IDs (e.g., 0 to 49)
for marker_id in range(50):
    # Create a blank image to draw the marker on
    marker_image = np.zeros((marker_size, marker_size), dtype=np.uint8)
    marker_image = aruco.drawMarker(aruco_dict, marker_id, marker_size, marker_image, 1)
    
    # Save the marker image to disk
    filename = os.path.join(output_dir, f"marker_{marker_id}.png")
    cv2.imwrite(filename, marker_image)
    
    # Display the marker
    cv2.imshow(f"Marker {marker_id}", marker_image)
    cv2.waitKey(100)  # Display each marker for 100ms
    cv2.destroyWindow(f"Marker {marker_id}")

cv2.destroyAllWindows()
print("Markers generated and saved in folder:", output_dir)
