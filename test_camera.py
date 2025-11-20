import cv2

cap = cv2.VideoCapture("/dev/video0")  # or the device your camera is on

if not cap.isOpened():
    print("Failed to open camera")
    exit()

ret, frame = cap.read()
if ret:
    cv2.imwrite("frame.jpg", frame)
    cv2.imshow("Camera", frame)
    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Failed to grab frame")
cap.release()
