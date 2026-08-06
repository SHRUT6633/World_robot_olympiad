import cv2
import numpy as np
import json

COLOR_CFG = {
    "red": [
        {"lower": [0, 50, 50], "upper": [10, 255, 255]},
        {"lower": [170, 50, 50], "upper": [180, 255, 255]},
    ],
    "blue": [
        {"lower": [100, 50, 50], "upper": [130, 255, 255]},
    ],
    "yellow": [
        {"lower": [20, 50, 50], "upper": [35, 255, 255]},
    ],
    "green": [
        {"lower": [40, 50, 50], "upper": [80, 255, 255]},
    ],
}

def load_color_calib(path="color_calib.json"):
    global COLOR_CFG
    try:
        with open(path) as f:
            COLOR_CFG = json.load(f)
        print(f"Loaded color calibration from {path}")
    except FileNotFoundError:
        print("No color_calib.json found, using defaults")

def detect_colors(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)
    masks = {}

    for color_name, ranges in COLOR_CFG.items():
        combined = None
        for r in ranges:
            lower = np.array(r["lower"], dtype=np.uint8)
            upper = np.array(r["upper"], dtype=np.uint8)
            mask = cv2.inRange(hsv, lower, upper)
            if combined is None:
                combined = mask
            else:
                combined = cv2.bitwise_or(combined, mask)
        masks[color_name] = combined

    return masks

def calibrate_from_frame(frame_bgr):
    hsv = cv2.cvtColor(frame_bgr, cv2.COLOR_BGR2HSV)

    def on_mouse(event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            pixel = hsv[y, x]
            print(f"HSV at ({x},{y}): {pixel}")

    cv2.imshow("Calibrate", frame_bgr)
    cv2.setMouseCallback("Calibrate", on_mouse)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    cap = cv2.VideoCapture(0)
    ret, frame = cap.read()
    if ret:
        load_color_calib()
        masks = detect_colors(frame)
        for name, mask in masks.items():
            cv2.imshow(name, mask)
        cv2.imshow("Original", frame)
        cv2.waitKey(0)
    cap.release()
    cv2.destroyAllWindows()
