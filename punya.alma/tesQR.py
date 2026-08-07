import cv2
import time
import numpy as np
from pyzbar.pyzbar import decode


# Start the webcam (0 is usually the built-in camera)
cap = cv2.VideoCapture(0)

# Check if the camera opens correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

camera_matrix = np.array([[ 1164.0743995447119, 0., 980.03650102852032], 
                          [0., 1158.2462609239342, 562.2397226536624], 
                          [0., 0., 1. ]]) 

object_points = np.array([
    [-2, 2, 0], [2, 2, 0], [2, -2, 0], [-2, -2, 0]
], dtype=float)

while True:
    # Read a single frame
    ret, frame = cap.read()

    # If frame reading is fail, break the loop
    if not ret:
        print("Error: Can't receive frame.")
        break
    
    decoded_objects = decode (frame)
    for obj in decoded_objects:
        data = obj.data.decode ("utf=8")
        print (data)
        print (obj.polygon)

        image_points = np.array(obj.polygon, dtype=float)
        pts = image_points
        print (image_points)
        

        #Sorts 4 points into: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)] # Top-Left has smallest sum
        rect[2] = pts[np.argmax(s)] # Bottom-Right has largest sum
        
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)] # Top-Right has smallest difference
        rect[3] = pts[np.argmax(diff)] # Bottom-Left has largest difference
        #return rect

        # Assuming no lens distortion
        dist_coeffs = np.array([ -0.081632802945328112, 0.029020913802887096,
       -0.0015049443976139964, -0.00088117174435944933,
       -0.053633582926546026 ])

        print (rect)


        # Solve for pose using SOLVEPNP_ITERATIVE
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points, rect, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        # Check if solvePnP was successful
        if success:
            print("Rotation Vector:\n", rotation_vector)
            print("Translation Vector:\n", translation_vector)
        else:
            print("solvePnP failed to find a solution.")

        time.sleep(0.01)

    # Display the live frame in a window
    cv2.imshow("Live Webcam", frame)

    # Press 'q' on the keyboard to stop the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the camera and close all windows
cap.release()
cv2.destroyAllWindows()

