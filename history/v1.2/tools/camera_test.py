from picamera2 import Picamera2
import time
import cv2

print("Initializing camera...")
cam = Picamera2()
cam.configure(cam.create_preview_configuration(
    main={"size": (640, 480), "format": "BGR888"}))
cam.start()
time.sleep(2)  # Critical: sensor warm-up

frame = cam.capture_array()
cv2.imwrite("first_frame.jpg", frame)
print(f"Captured 640x480 frame, saved to first_frame.jpg")
cam.stop()
cam.close()
