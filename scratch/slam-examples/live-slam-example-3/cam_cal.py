import cv2
import numpy as np
import glob

# Settings
checkerboard_size = (6, 9)  # number of inner corners per row and column
square_size = 0.026  # size of each square in the checkerboard pattern (e.g., 1 cm)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# Prepare object points
objp = np.zeros((checkerboard_size[0] * checkerboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:checkerboard_size[0], 0:checkerboard_size[1]].T.reshape(-1, 2)
objp *= square_size  # multiply by the size of squares

objpoints = []  # 3D points in real world space
imgpoints = []  # 2D points in image plane

# Load calibration images
images = glob.glob('calibration_images/*.jpg')

for image_path in images:
    img = cv2.imread(image_path)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    
    # Find the chessboard corners
    ret, corners = cv2.findChessboardCorners(gray, checkerboard_size, None)
    
    # If found, add object points and image points
    if ret:
        objpoints.append(objp)
        corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        imgpoints.append(corners_refined)

# Perform calibration
ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Print the re-projection error
print("\nRe-projection Error (ret):")
print(f"{ret:.6f}")

# Print the camera matrix (Intrinsic parameters)
print("\nCamera Matrix (mtx):")
print(np.array2string(mtx, formatter={'float_kind':lambda x: f"{x:.6f}"}))

# Print the distortion coefficients
print("\nDistortion Coefficients (dist):")
print(np.array2string(dist, formatter={'float_kind':lambda x: f"{x:.6f}"}))

# Print the rotation vectors for each calibration image
print("\nRotation Vectors (rvecs):")
for i, rvec in enumerate(rvecs):
    print(f"Image {i+1} Rotation Vector:")
    print(np.array2string(rvec, formatter={'float_kind':lambda x: f"{x:.6f}"}))

# Print the translation vectors for each calibration image
print("\nTranslation Vectors (tvecs):")
for i, tvec in enumerate(tvecs):
    print(f"Image {i+1} Translation Vector:")
    print(np.array2string(tvec, formatter={'float_kind':lambda x: f"{x:.6f}"}))

# Save parameters to a YAML file with custom format
output_file = 'imx219-83.yaml'

with open(output_file, 'w') as f:
    # Write YAML structure with comments and format
    f.write("%YAML:1.0\n\n")
    f.write("#--------------------------------------------------------------------------------------------\n")
    f.write("# System config\n")
    f.write("#--------------------------------------------------------------------------------------------\n\n")
    
    # File version
    f.write("File.version: \"1.0\"\n\n")
    
    # Camera parameters
    f.write("#--------------------------------------------------------------------------------------------\n")
    f.write("# Camera Parameters. Adjust them!\n")
    f.write("#--------------------------------------------------------------------------------------------\n\n")
    f.write("Camera.type: \"PinHole\"\n\n")
    
    # Camera intrinsic parameters
    f.write(f"Camera1.fx: {mtx[0, 0]}\n")
    f.write(f"Camera1.fy: {mtx[1, 1]}\n")
    f.write(f"Camera1.cx: {mtx[0, 2]}\n")
    f.write(f"Camera1.cy: {mtx[1, 2]}\n\n")
    
    # Camera distortion coefficients
    f.write(f"Camera1.k1: {dist[0, 0]}\n")
    f.write(f"Camera1.k2: {dist[0, 1]}\n")
    f.write(f"Camera1.p1: {dist[0, 2]}\n")
    f.write(f"Camera1.p2: {dist[0, 3]}\n")
    f.write(f"Camera1.p3: {dist[0, 4]}\n\n")

    # Optional camera resolutions
    f.write("Camera.width: 640\n")
    f.write("Camera.height: 480\n\n")
    f.write("Camera.newWidth: 600\n")
    f.write("Camera.newHeight: 350\n\n")
    
    # Other camera parameters
    f.write("Camera.fps: 120\n")
    f.write("Camera.RGB: 1\n\n")
    
    # ORB Parameters
    f.write("#--------------------------------------------------------------------------------------------\n")
    f.write("# ORB Parameters\n")
    f.write("#--------------------------------------------------------------------------------------------\n\n")
    f.write("ORBextractor.nFeatures: 1000\n")
    f.write("ORBextractor.scaleFactor: 1.2\n")
    f.write("ORBextractor.nLevels: 8\n")
    f.write("ORBextractor.iniThFAST: 20\n")
    f.write("ORBextractor.minThFAST: 7\n\n")
    
    # Viewer Parameters
    f.write("#--------------------------------------------------------------------------------------------\n")
    f.write("# Viewer Parameters\n")
    f.write("#---------------------------------------------------------------------------------------------\n\n")
    f.write("Viewer.KeyFrameSize: 0.05\n")
    f.write("Viewer.KeyFrameLineWidth: 1.0\n")
    f.write("Viewer.GraphLineWidth: 0.9\n")
    f.write("Viewer.PointSize: 2.0\n")
    f.write("Viewer.CameraSize: 0.08\n")
    f.write("Viewer.CameraLineWidth: 3.0\n")
    f.write("Viewer.ViewpointX: 0.0\n")
    f.write("Viewer.ViewpointY: -0.7\n")
    f.write("Viewer.ViewpointZ: -1.8\n")
    f.write("Viewer.ViewpointF: 500.0\n")

print(f"Calibration parameters have been saved to {output_file} in the specified format.")