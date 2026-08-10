import cv2
import time
import numpy as np
from pyzbar.pyzbar import decode

# Start the webcam (0 is usually the built-in camera)
cap = cv2.VideoCapture(0)

# Paksa resolusi kamera ke 1080p (Sesuai dengan camera_matrix kalibrasi)
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 1920)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 1080)

# Check if the camera opens correctly
if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

# Buat window yang ukurannya bisa di-resize agar tampilan tidak meluber keluar layar
cv2.namedWindow("Live Webcam", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Live Webcam", 960, 540)

camera_matrix = np.array([
    [1164.0743995447119, 0., 980.03650102852032], 
    [0., 1158.2462609239342, 562.2397226536624], 
    [0., 0., 1.]
], dtype=np.float32) 

object_points = np.array([
    [-2, 2, 0], [2, 2, 0], [2, -2, 0], [-2, -2, 0]
], dtype=np.float32)

dist_coeffs = np.array([
    -0.081632802945328112, 0.029020913802887096,
    -0.0015049443976139964, -0.00088117174435944933,
    -0.053633582926546026
], dtype=np.float32)

while True:
    ret, frame = cap.read()
    if not ret:
        print("Error: Can't receive frame.")
        break
    
    decoded_objects = decode(frame)
    for obj in decoded_objects:
        data = obj.data.decode("utf-8")
        image_points = np.array(obj.polygon, dtype=np.float32)
        pts = image_points

        # Sorts 4 points into: Top-Left, Top-Right, Bottom-Right, Bottom-Left
        rect = np.zeros((4, 2), dtype=np.float32)
        s = pts.sum(axis=1)
        rect[0] = pts[np.argmin(s)] 
        rect[2] = pts[np.argmax(s)] 
        
        diff = np.diff(pts, axis=1)
        rect[1] = pts[np.argmin(diff)] 
        rect[3] = pts[np.argmax(diff)] 

        # Solve for pose using SOLVEPNP_IPPE_SQUARE
        success, rotation_vector, translation_vector = cv2.solvePnP(
            object_points, rect, camera_matrix, dist_coeffs, flags=cv2.SOLVEPNP_IPPE_SQUARE
        )

        if success:
            # 1. Gambar Sumbu 3D Axis
            axis_length = 3.0 
            cv2.drawFrameAxes(frame, camera_matrix, dist_coeffs, rotation_vector, translation_vector, axis_length, thickness=3)

            # 2. Ambil nilai X, Y, Z
            tx = translation_vector[0][0]
            ty = translation_vector[1][0]
            tz = translation_vector[2][0]

            # 3. Tampilkan Teks X, Y, Z Lengkap di Layar
            xyz_text = f"X: {tx:.1f} | Y: {ty:.1f} | Z: {tz:.1f} cm"
            corner_pt = (int(rect[0][0]), int(rect[0][1]) - 15)
            
            # Teks warna hijau dengan background tipis/penebalan hitam biar gampang dibaca
            cv2.putText(frame, xyz_text, corner_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4) # Outline
            cv2.putText(frame, xyz_text, corner_pt, cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2) # Text Utama

        time.sleep(0.01)

    # Display the live frame in a window
    cv2.imshow("Live Webcam", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()