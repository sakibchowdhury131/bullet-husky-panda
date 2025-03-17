import cv2
import numpy as np
x1, y1, z1 = 3, 4, 5
x2, y2, z2 = 4, 5, 6
x3, y3, z3 = 2, 6, 4
x4, y4, z4 = 2, 4, 5
x5, y5, z5 = 6, 4, 5
x6, y6, z6 = 2, 6, 5

u1, v1 = 200, 300
u2, v2 = 300, 300
u3, v3 = 400, 300
u4, v4 = 200, 200
u5, v5 = 350, 250
u6, v6 = 250, 250

fx = 609.4323
fy = 609.5173
cx = 315.48343
cy = 247.94066


# 1. Define known 3D points in the global (court) coordinate system
object_points = np.array([
    [x1, y1, z1],  # Point 1 in global coordinates
    [x2, y2, z2],  # Point 2 in global coordinates
    [x3, y3, z3],  # Point 3 in global coordinates
    [x4, y4, z4],  # Point 4 in global coordinates
    [x5, y5, z5],  # Point 4 in global coordinates
    [x6, y6, z6],  # Point 4 in global coordinates
], dtype=np.float32)

# 2. Corresponding 2D pixel coordinates detected in the camera frame
image_points = np.array([
    [u1, v1],  # Pixel coordinates of point 1
    [u2, v2],  # Pixel coordinates of point 2
    [u3, v3],  # Pixel coordinates of point 3
    [u4, v4],  # Pixel coordinates of point 4
    [u5, v5],  # Pixel coordinates of point 4
    [u6, v6],  # Pixel coordinates of point 4
], dtype=np.float32)

# 3. Camera intrinsic parameters (can be obtained from calibration)
camera_matrix = np.array([
    [fx,  0, cx],
    [ 0, fy, cy],
    [ 0,  0,  1]
], dtype=np.float32)

# Assuming no lens distortion (or replace with actual distortion coefficients)
dist_coeffs = np.zeros((4, 1), dtype=np.float32)

# 4. SolvePnP to estimate rotation and translation
success, rvec, tvec = cv2.solvePnP(object_points, image_points, camera_matrix, dist_coeffs)

if success:
    # Convert rotation vector to a 3x3 rotation matrix
    R, _ = cv2.Rodrigues(rvec)
    t = tvec.reshape(3)  # Convert to (3,) shape for easier computation

    print("Rotation Matrix (R):\n", R)
    print("Translation Vector (t):\n", t)
