import cv2
import numpy as np
import glob
import pickle
import os
# Calibration settings
CHECKERBOARD = (9, 6)  # Change this to match your checkerboard pattern (internal corners, not squares)
SQUARE_SIZE = 25  # Square size in mm (optional, only needed for real-world scaling)
IMAGE_FOLDER = "cheackerImages/Try3"  # Folder containing calibration images
RECTIFIED_FOLDER = "cheackerImages/rectified3"  # Folder to save rectified images
# Prepare object points (3D points in real-world space)
objp = np.zeros((CHECKERBOARD[0] * CHECKERBOARD[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:CHECKERBOARD[0], 0:CHECKERBOARD[1]].T.reshape(-1, 2)
objp *= SQUARE_SIZE  # Scale to real-world units

# Arrays to store object points and image points
objpoints = []  # 3D points
imgpoints = []  # 2D points

# Load images
images = glob.glob(f"{IMAGE_FOLDER}/*.jpg")  # Modify for your image format
print(images)
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, CHECKERBOARD, None)
    
    if ret:
        objpoints.append(objp)
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), 
                                    criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, CHECKERBOARD, corners2, ret)
        cv2.imshow(f'Corners in {fname}', img)
        cv2.waitKey(500)  # Display for 500ms
        
    else:
        print(f"Checkerboard not found in {fname}")

cv2.destroyAllWindows()

# Perform camera calibration
ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Save the calibration results
calibration_data = {
    "camera_matrix": camera_matrix,
    "dist_coeffs": dist_coeffs,
    "rvecs": rvecs,
    "tvecs": tvecs
}
with open("camera_calibration.pkl", "wb") as f:
    pickle.dump(calibration_data, f)

print("Camera matrix:")
print(camera_matrix)
print("\nDistortion coefficients:")
print(dist_coeffs)

# Create rectified image folder if it doesn't exist
if not os.path.exists(RECTIFIED_FOLDER):
    os.makedirs(RECTIFIED_FOLDER)

# Rectify images using the calibration data
for fname in images:
    img = cv2.imread(fname)
    h, w = img.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(camera_matrix, dist_coeffs, (w, h), 1, (w, h))
    
    # Undistort image
    dst = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)
    
    # Crop the image based on ROI
    x, y, w, h = roi
    dst = dst[y:y+h, x:x+w]
    
    # Save rectified image
    rectified_fname = os.path.join(RECTIFIED_FOLDER, os.path.basename(fname))
    cv2.imwrite(rectified_fname, dst)

print("Rectified images saved in 'rectified' folder.")
