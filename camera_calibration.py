import numpy as np
import cv2 as cv
import glob
import os

# measured side length of a chessboard square in mm
square_size = 22.15

# input the amount of rows and columns that corners are visible
row_size = 7
column_size = 6

# create zero array of 3D points for each corner of chessboard (32bits more reliable for opencv)
real_board_points = np.zeros((row_size*column_size,3), np.float32)

# assign coords to corners, top left corner is origin, x to the right is + and y down is +, z axis = 0
real_board_points[:,:2] = np.mgrid[0:row_size,0:column_size].T.reshape(-1,2) * square_size


# Arrays to store object points and image points from all the images.
objpoints = [] # 3d points in real world space
imgpoints = [] # 2d points in image plane.

# termination criteria: stop if iterations = 30 or pixel accuracy is less than epsilon value(0.001)
criteria = (cv.TERM_CRITERIA_EPS + cv.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# window size of subpixel estimate for corner coords
pixel_search_area = (11,11)

# deadzone for subpixel estimate for corner coords (set to None)
deadzone = (-1,-1)

path = "chessboard_photos/Cameron's_iphone"
images = glob.glob(f"{path}/*.jpeg")

for image in images:

    filename = os.path.basename(image)

    img = cv.imread(image)
    gray = cv.cvtColor(img, cv.COLOR_BGR2GRAY)

    corners_found, corner_points = cv.findChessboardCorners(gray, (row_size,column_size), None)

    if corners_found:

        print(f'{filename} is a good image')
        

        objpoints.append(real_board_points)

        #looks at the color gradient and makes the corner point estimate more accurate
        refined_corner_points =  cv.cornerSubPix(gray,corner_points, pixel_search_area, deadzone, criteria)
        imgpoints.append(refined_corner_points)

        # Draw and display the corners in a resized frame for clarity
        cv.drawChessboardCorners(img, (row_size,column_size), refined_corner_points, corners_found)
        scale = 0.2
        resized = cv.resize(img, None, fx=scale, fy=scale)
        cv.imshow('img', resized)
        cv.waitKey(0) # manually click to go to next image
    else:

        print(f'{filename} is a bad image')


error, intrinsic_matrix, distortion_coefficents, orientation, position = cv.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

# Extract intrinsic parameters from the calibration matrix
fx = intrinsic_matrix[0, 0]
fy = intrinsic_matrix[1, 1]
Uo = intrinsic_matrix[0, 2]
Vo = intrinsic_matrix[1, 2]

print("\n================ CAMERA INTRINSICS ==================")
print(f"RMS reprojection error:        {error:.4f}")
print(f"Focal length fx:               {fx:.4f} pixels")
print(f"Focal length fy:               {fy:.4f} pixels")
print(f"Principal point cx:            {Uo:.4f} pixels")
print(f"Principal point cy:            {Vo:.4f} pixels")


save_or_not = input("Do you want to save the calibration values (yes or no)? ").strip().lower()

if save_or_not == "yes":
    np.savez(
        'camera_calibration_data.npz',
        camera_matrix=intrinsic_matrix,
        dist_coeffs=distortion_coefficents,
        rvecs=orientation,
        tvecs=position
    )
    print("\n✅ Calibration data saved to 'camera_calibration_data.npz'")
else:
    print("\n❎ Calibration data not saved.")