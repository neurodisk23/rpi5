import cv2
import os
from datetime import datetime

# Create a directory to save frames
output_dir = 'saved_frames'
os.makedirs(output_dir, exist_ok=True)

# Open the webcam (0 is the default camera)
cap = cv2.VideoCapture(0)

if not cap.isOpened():
    print("Error: Could not open webcam.")
    exit()

try:
    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()
        if not ret:
            print("Error: Could not read frame.")
            break

        # Display the frame in a window
        cv2.imshow('Webcam', frame)

        # Get current timestamp with sub-seconds
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S') + f"_{int(datetime.now().microsecond / 1000):03d}"
        frame_filename = os.path.join(output_dir, f'frame_{timestamp}.jpg')

        # Save the current frame with timestamp
        cv2.imwrite(frame_filename, frame)
        print(f'Saved {frame_filename}')

        # Press 'q' to exit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    # Release the webcam and close windows
    cap.release()
    cv2.destroyAllWindows()
