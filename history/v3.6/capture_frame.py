import cv2
import numpy as np
import threading
import queue
import time
import os
from picamera2 import Picamera2

WARMUP_FRAMES = 5
FRAME_WIDTH = 640
FRAME_HEIGHT = 480
FRAME_RATE = 60

os.makedirs("capture", exist_ok=True)

cam = Picamera2()
config = cam.create_video_configuration(
    main={"size": (FRAME_WIDTH, FRAME_HEIGHT), "format": "RGB888"},
    buffer_count=2,
)
cam.configure(config)
cam.set_controls({"FrameRate": FRAME_RATE})
cam.start()

for _ in range(WARMUP_FRAMES):
    cam.capture_array("main")

write_queue = queue.Queue(maxsize=30)

def writer():
    while True:
        buf, timestamp = write_queue.get()
        filename = f"capture/frame_{timestamp:.6f}.jpg"
        cv2.imwrite(filename, buf)
        write_queue.task_done()

writer_thread = threading.Thread(target=writer, daemon=True)
writer_thread.start()

running = True
frame_count = 0
start_time = time.perf_counter()

while running:
    frame = cam.capture_array("main")
    t = time.perf_counter() - start_time

    if frame_count % 2 == 0:
        try:
            write_queue.put_nowait((frame, t))
        except queue.Full:
            pass

    frame_count += 1

    if cv2.waitKey(1) & 0xFF == ord('q'):
        running = False

write_queue.join()
cam.stop()
