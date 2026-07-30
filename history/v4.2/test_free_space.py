import cv2
from free_space import FreeSpaceDetect


def test_free_space():
    detector = FreeSpaceDetect()
    cap = cv2.VideoCapture(0)

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        result = detector.process(frame)
        cv2.imshow("Free Space Mask", result["mask"])
        print(f"Free space: {result['free_space_pct']:.1f}%")

        if cv2.waitKey(1) & 0xFF == ord("q"):
            break

    cap.release()
    cv2.destroyAllWindows()
