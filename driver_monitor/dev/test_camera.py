import cv2
import time

# 0번 = 기본 내장 웹캠. 안 되면 1, 2로 바꿔보세요.
CAM_INDEX = 0

# Windows에서는 CAP_DSHOW를 주면 카메라가 훨씬 빨리 열립니다.
cap = cv2.VideoCapture(CAM_INDEX, cv2.CAP_DSHOW)

if not cap.isOpened():
    print(f"[ERROR] {CAM_INDEX}번 카메라를 열 수 없습니다.")
    print("- 다른 프로그램(Zoom, 카메라 앱)이 쓰고 있는지 확인")
    print("- 설정 > 개인정보 > 카메라 권한 확인")
    print("- CAM_INDEX를 1이나 2로 바꿔보세요")
    raise SystemExit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

print("[INFO] 카메라 시작. 종료하려면 창을 클릭하고 q를 누르세요.")

prev = time.time()
fps = 0.0

while True:
    ok, frame = cap.read()
    if not ok:
        print("[WARN] 프레임을 읽지 못했습니다.")
        break

    # 거울처럼 좌우 반전 (본인 모습 볼 때 자연스러움)
    frame = cv2.flip(frame, 1)

    # FPS 계산
    now = time.time()
    dt = now - prev
    prev = now
    if dt > 0:
        fps = 0.9 * fps + 0.1 * (1.0 / dt)   # 부드럽게 평균

    cv2.putText(frame, f"FPS: {fps:.1f}", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("Camera Test", frame)

    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print(f"[INFO] 종료. 마지막 FPS: {fps:.1f}")
