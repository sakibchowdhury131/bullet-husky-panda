import cv2
import numpy as np
import cv2.aruco as aruco

# Define the ArUco dictionary
aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_6X6_250)

# Create a ChArUco board (checkerboard with ArUco markers)
board = aruco.CharucoBoard_create(
    squaresX=5, squaresY=7,  # Grid size (cols, rows)
    squareLength=0.04,  # Square size in meters
    markerLength=0.03,  # Marker size in meters
    dictionary=aruco_dict
)

# Save the board as an image
board_image = board.draw((1000, 1400))
cv2.imwrite("charuco_board.png", board_image)

cv2.imshow("ChArUco Board", board_image)
cv2.waitKey(0)
cv2.destroyAllWindows()
