import cv2
import numpy as np


"""
This script takes a video and convert it to vextor of x,y at the phase """
# Initialize video capture
cap = cv2.VideoCapture(0)

# Initialize variables for previous frame
prev_frame = None
prev_pts = None
prev_x = 0
prev_y = 0

# Parameters for Lucas-Kanade optical flow
lk_params = dict(winSize=(15, 15),
                 maxLevel=2,
                 criteria=(cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 0.03))

# Parameters for smoothing
alpha = 0.2  # Smoothing factor

# Noise canceling threshold
movement_threshold = 5.0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Convert frame to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply Gaussian blur to reduce noise
    gray_blur = cv2.GaussianBlur(gray, (5, 5), 0)

    # If this is not the first frame, calculate the optical flow
    if prev_frame is not None and prev_pts is not None:
        # Calculate optical flow using Lucas-Kanade method
        next_pts, status, _ = cv2.calcOpticalFlowPyrLK(prev_frame, gray_blur, prev_pts, None, **lk_params)

        # Select good points
        good_new = next_pts[status == 1]
        good_old = prev_pts[status == 1]

        if len(good_new) > 0 and len(good_old) > 0:
            # Calculate the average movement
            movement = np.mean(good_new - good_old, axis=0)

            # Extract x and y components of the average movement
            x_movement = movement[0]
            y_movement = movement[1]

            # Apply smoothing
            if prev_x != 0 and prev_y != 0:
                x_movement = alpha * x_movement + (1 - alpha) * prev_x
                y_movement = alpha * y_movement + (1 - alpha) * prev_y

            # Apply noise canceling
            if abs(x_movement) < movement_threshold:
                x_movement = 0
            if abs(y_movement) < movement_threshold:
                y_movement = 0

            # Update previous x and y positions
            prev_x = x_movement
            prev_y = y_movement

            # Print the movement
            print(x_movement, y_movement)
    # Store the current frame and points for the next iteration
    prev_frame = gray_blur.copy()
    prev_pts = cv2.goodFeaturesToTrack(gray_blur, maxCorners=100, qualityLevel=0.01, minDistance=10)

    # Break the loop if 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the video capture
cap.release()
cv2.destroyAllWindows()
