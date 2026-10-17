import cv2

WINDOW_NAME = "Camera"

cap = cv2.VideoCapture(1)

if not cap.isOpened():
    print("웹캠을 열 수 없습니다.")
    exit()

cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)

while True:
    ret, frame = cap.read()
    if not ret:
        break

    cv2.imshow(WINDOW_NAME, frame)

    key = cv2.waitKey(1) & 0xFF
    window_is_open = cv2.getWindowProperty(
        WINDOW_NAME,
        cv2.WND_PROP_VISIBLE,
    ) >= 1
    if not window_is_open or key == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
