import cv2
import numpy as np
from qrdet import QRDetector
import threading
import queue
import time


def detection_worker(detector, frame_queue, result_queue):
    """Background thread that pulls the latest frame from the queue."""
    while True:
        # Blocks until a frame is available
        frame_to_process = frame_queue.get()

        # Stop signal
        if frame_to_process is None:
            break

        # 1. Run detection
        detections = detector.detect(image=frame_to_process, is_bgr=True)

        new_corners = []
        for detection in detections:
            polygon = np.array(detection["polygon_xy"], dtype=np.int32)
            perimeter = cv2.arcLength(polygon, True)
            corners = cv2.approxPolyDP(polygon, 0.05 * perimeter, True)
            new_corners.append(corners)

        # 2. Push the result back to the main thread
        # Clear the old result if the main thread hasn't picked it up yet
        try:
            result_queue.get_nowait()
        except queue.Empty:
            pass

        result_queue.put(new_corners)


def main():
    # Queues with size 1 to ensure we only ever hold the absolute latest data
    frame_queue = queue.Queue(maxsize=1)
    result_queue = queue.Queue(maxsize=1)

    detector = QRDetector(model_size="n", conf_th=0.5)

    detect_thread = threading.Thread(
        target=detection_worker, args=(detector, frame_queue, result_queue), daemon=True
    )
    detect_thread.start()

    cap = cv2.VideoCapture("/home/vane/Videos/Webcam/2026-08-20-154425.webm")
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*"MJPG"))
    cap.set(cv2.CAP_PROP_FPS, 5)

    if not cap.isOpened():
        print("Error: Could not open the live camera stream.")
        return

    print("Live stream started. Press 'q' to exit.")

    current_corners_to_draw = []

    while True:
        time.sleep(0.02)
        ret, frame = cap.read()
        if not ret:
            break

        # 1. Feed the newest frame to the worker.
        # If the worker hasn't finished the last frame, drop the old queued frame.
        try:
            frame_queue.get_nowait()
        except queue.Empty:
            pass
        frame_queue.put(
            frame.copy()
        )  # Pass a copy so main thread drawing doesn't tear the image

        # 2. Check if the worker has output new corners.
        # If not, it just keeps drawing the last known corners.
        try:
            current_corners_to_draw = result_queue.get_nowait()
        except queue.Empty:
            pass

        # 3. Draw bounding boxes
        for corners in current_corners_to_draw:
            cv2.polylines(frame, [corners], True, (0, 255, 0), 2)

        cv2.imshow("Live Cam QR Detection", frame)

        if cv2.waitKey(1) & 0xFF == ord("q"):
            # Send stop signal to worker
            try:
                frame_queue.get_nowait()
            except queue.Empty:
                pass
            frame_queue.put(None)
            break

    cap.release()
    cv2.destroyAllWindows()
    detect_thread.join()


if __name__ == "__main__":
    main()
