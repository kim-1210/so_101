import cv2

WINDOW_NAME = "Webcam"
WINDOW_WIDTH = 640
WINDOW_HEIGHT = 480


def main():
    # 0은 컴퓨터에 연결된 첫 번째 웹캠입니다.
    camera = cv2.VideoCapture(1)

    if not camera.isOpened():
        print("웹캠을 열 수 없습니다.")
        return

    print("웹캠을 시작합니다. 종료하려면 q 키를 누르세요.")
    cv2.namedWindow(WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(WINDOW_NAME, WINDOW_WIDTH, WINDOW_HEIGHT)

    try:
        while True:
            success, frame = camera.read()
            if not success:
                print("웹캠 영상을 읽을 수 없습니다.")
                break

            cv2.imshow(WINDOW_NAME, frame)

            # q 키를 누르면 반복문을 종료합니다.
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        camera.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
