import cv2
import numpy as np
from collections import namedtuple
from concurrent.futures import ThreadPoolExecutor

Blob = namedtuple("Blob", ["color", "x", "y", "width", "height", "area"])
MIN_AREA = 200

executor = ThreadPoolExecutor(max_workers=4)

def _find_blobs_single(mask, color_name):
    kernel = np.ones((3, 3), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)

    small = cv2.resize(mask, (0, 0), fx=0.5, fy=0.5,
                       interpolation=cv2.INTER_NEAREST)
    contours, _ = cv2.findContours(small, cv2.RETR_EXTERNAL,
                                   cv2.CHAIN_APPROX_SIMPLE)

    blobs = []
    for c in contours:
        area = cv2.contourArea(c) * 4
        if area < MIN_AREA:
            continue

        x, y, w, h = cv2.boundingRect(c)
        x *= 2; y *= 2; w *= 2; h *= 2

        if w > h:
            continue

        blobs.append(Blob(color=color_name, x=x, y=y,
                          width=w, height=h, area=area))
    return blobs

def find_blobs(masks):
    futures = []
    for color_name, mask in masks.items():
        futures.append(executor.submit(_find_blobs_single, mask, color_name))

    all_blobs = []
    for f in futures:
        all_blobs.extend(f.result())
    return all_blobs

def draw_blobs(frame, blobs):
    for b in blobs:
        color_map = {
            "red": (0, 0, 255),
            "blue": (255, 0, 0),
            "yellow": (0, 255, 255),
            "green": (0, 255, 0),
        }
        c = color_map.get(b.color, (255, 255, 255))
        cv2.rectangle(frame, (b.x, b.y), (b.x + b.width, b.y + b.height), c, 2)
        cv2.putText(frame, b.color, (b.x, b.y - 5),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, c, 1)
    return frame

if __name__ == "__main__":
    from color_detect import detect_colors, load_color_calib

    cap = cv2.VideoCapture(0)
    load_color_calib()

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        masks = detect_colors(frame)
        blobs = find_blobs(masks)
        frame = draw_blobs(frame, blobs)

        cv2.imshow("Blob Detection", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
