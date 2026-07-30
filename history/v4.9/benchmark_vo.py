import cv2
import time
from visual_odometry import VisualOdometry


def benchmark():
    vo = VisualOdometry()
    cap = cv2.VideoCapture(0)

    frame_count = 0
    total_time = 0.0

    while frame_count < 100:
        ret, frame = cap.read()
        if not ret:
            break

        t0 = time.perf_counter()
        result = vo.process(frame)
        t1 = time.perf_counter()
        total_time += (t1 - t0)
        frame_count += 1

        if result["moved"]:
            print(f"Frame {frame_count}: dx={result['dx']:.2f}, dy={result['dy']:.2f}")

    cap.release()
    avg_fps = frame_count / total_time
    print(f"Average FPS: {avg_fps:.1f}")
    print(f"Average frame time: {total_time / frame_count * 1000:.1f} ms")


if __name__ == "__main__":
    benchmark()
