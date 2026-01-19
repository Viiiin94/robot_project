import cv2
import mediapipe as mp
import face_recognition
import serial
import time
import numpy as np
import os
import struct
from PIL import Image
from multiprocessing import Process, Queue


# =========================
# 화면 에러 방지 (X 서버)
# =========================
if "DISPLAY" not in os.environ:
    os.environ["DISPLAY"] = ":0"


# =========================
# [설정 구역]
# =========================

# 1. 이미지 경로
PATH_BASE = "/home/user17/face_tracking/"
IMG_NORMAL = "normal.jpg"
IMG_HAPPY = "happy.jpg"
IMG_STOP = "stop.jpg"

# 2. UART 설정
MOTOR_PORT = "/dev/ttyS0"
MOTOR_BAUD = 115200

LCD_PORT = "/dev/ttyAMA4"
LCD_BAUD = 460800
LCD_W, LCD_H = 240, 320

# 3. 주행 파라미터
SERVO_STEP = -1
CENTER_MIN = 87
CENTER_MAX = 93

FWD_SPEED_BASE = 100
TANK_TURN_SPEED = 100

DEADZONE = 10
TARGET_FACE_SIZE = 0.15


# =========================================================
# [Process 2] LCD 담당 프로세스 (표정 출력 전용)
# =========================================================
def lcd_worker(cmd_q: Queue, ack_q: Queue):

    try:
        ser_lcd = serial.Serial(LCD_PORT, LCD_BAUD, timeout=1)
        print("📺 [LCD] 연결 성공")
    except Exception as e:
        print("❌ [LCD] 연결 실패:", e)
        return

    current_img_name = ""

    while True:

        # 새 표정 명령이 있을 때만 처리
        if not cmd_q.empty():
            img_name = cmd_q.get()

            # 동일 표정이면 전송 생략 (중복 방지)
            if img_name == current_img_name:
                ack_q.put("DONE")
                continue

            current_img_name = img_name
            full_path = os.path.join(PATH_BASE, img_name)

            if not os.path.exists(full_path):
                print(f"❌ [LCD] 파일 없음: {full_path}")
                ack_q.put("DONE")
                continue

            try:
                # 이미지 로드 및 RGB 변환
                img = Image.open(full_path).convert("RGB")

                # NORMAL 상태만 상하 반전 (카메라 기준 보정)
                if img_name == IMG_NORMAL:
                    img = img.transpose(Image.FLIP_TOP_BOTTOM)

                img = img.resize((LCD_W, LCD_H))
                pixel_data = list(img.getdata())

                buffer = bytearray()

                # RGB888 → RGB565 변환
                def to_rgb565(r, g, b):
                    return ((r & 0xF8) << 8) | ((g & 0xFC) << 3) | (b >> 3)

                for r, g, b in pixel_data:
                    buffer.extend(struct.pack(">H", to_rgb565(r, g, b)))

                # FPGA / LCD 전송
                ser_lcd.reset_output_buffer()
                time.sleep(0.02)
                ser_lcd.write(buffer)
                ser_lcd.flush()

                print(f"📺 [LCD] 표정 변경 완료: {img_name}")

            except Exception as e:
                print(f"⚠️ [LCD] 전송 오류: {e}")

            # 전송 완료 ACK
            ack_q.put("DONE")

        time.sleep(0.05)  # CPU 점유 방지


# =========================================================
# [Process 1] 메인 제어 로직
# =========================================================
def main():

    # ---------- IPC 큐 ----------
    lcd_cmd_q = Queue()
    lcd_ack_q = Queue()

    p = Process(target=lcd_worker, args=(lcd_cmd_q, lcd_ack_q))
    p.start()

    # ---------- 모터 UART ----------
    try:
        ser_motor = serial.Serial(MOTOR_PORT, MOTOR_BAUD, timeout=0.01)
        print("✅ [Main] 모터 연결 성공")
    except:
        ser_motor = None
        print("⚠️ [Main] 모터 없음 (테스트 모드)")

    # ---------- AI 모델 ----------
    mp_face = mp.solutions.face_detection.FaceDetection(
        model_selection=0,
        min_detection_confidence=0.5
    )

    mp_hands = mp.solutions.hands.Hands(
        max_num_hands=1,
        min_detection_confidence=0.7
    )

    mp_drawing = mp.solutions.drawing_utils

    # ---------- 상태 변수 ----------
    is_owner = False
    cmd_mode = "STOP"
    last_emotion_cmd = ""

    loss_cnt = 0
    servo_angle = 90

    pTime = 0

    # ---------- 카메라 ----------
    cap = cv2.VideoCapture(0)
    cap.set(3, 320)
    cap.set(4, 240)

    # 초기 표정
    lcd_cmd_q.put(IMG_NORMAL)
    last_emotion_cmd = IMG_NORMAL

    print("🚀 로봇 가동 시작 (q: 종료)")

    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        # FPS 계산
        cTime = time.time()
        fps = 1 / (cTime - pTime) if cTime != pTime else 0
        pTime = cTime

        frame = cv2.flip(frame, 1)
        h, w, _ = frame.shape
        center_x = w // 2

        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # 모터 기본값 (정지)
        l_speed = r_speed = dir_bit = 0
        target_emotion = last_emotion_cmd

        # ---------------- 손 인식 (STOP) ----------------
        results_hand = mp_hands.process(rgb)

        if is_owner and results_hand.multi_hand_landmarks:
            for hand_lms in results_hand.multi_hand_landmarks:
                mp_drawing.draw_landmarks(
                    frame, hand_lms, mp.solutions.hands.HAND_CONNECTIONS
                )

                fingers = []
                for idx in [8, 12, 16, 20]:
                    fingers.append(
                        1 if hand_lms.landmark[idx].y <
                        hand_lms.landmark[idx - 2].y else 0
                    )

                if sum(fingers) >= 3:
                    cmd_mode = "STOP"
                    target_emotion = IMG_STOP
                else:
                    cmd_mode = "FOLLOW"

        # ---------------- 얼굴 인식 ----------------
        results_face = mp_face.process(rgb)
        dist_score = 0

        if results_face.detections:
            loss_cnt = 0

            target = max(
                results_face.detections,
                key=lambda d: d.location_data.relative_bounding_box.width
            )

            bbox = target.location_data.relative_bounding_box
            x = int(bbox.xmin * w)
            bw = int(bbox.width * w)

            face_cx = x + bw // 2
            face_size_ratio = bbox.width

            # 서보 제어
            err_x = face_cx - center_x
            if abs(err_x) > DEADZONE:
                servo_angle += SERVO_STEP if err_x < 0 else -SERVO_STEP
                servo_angle = max(0, min(180, servo_angle))

            # 몸통 회전 판단
            turn_action = 0
            if servo_angle < CENTER_MIN:
                turn_action = -1
            elif servo_angle > CENTER_MAX:
                turn_action = 1

            dist_score = int(100 / bbox.width)

            # 박스 표시
            box_color = (0, 255, 0) if is_owner else (0, 0, 255)
            cv2.rectangle(
                frame,
                (x, int(bbox.ymin * h)),
                (x + bw, int((bbox.ymin + bbox.height) * h)),
                box_color,
                2
            )

            # 주인 추적
            if is_owner and cmd_mode == "FOLLOW":
                target_emotion = IMG_HAPPY

                if turn_action != 0:
                    l_speed = r_speed = TANK_TURN_SPEED
                    dir_bit = 10 if turn_action == -1 else 5
                elif face_size_ratio < TARGET_FACE_SIZE:
                    l_speed = r_speed = FWD_SPEED_BASE
                    dir_bit = 9
            else:
                if turn_action == -1:
                    l_speed = r_speed = 100
                    dir_bit = 10
                elif turn_action == 1:
                    l_speed = r_speed = 100
                    dir_bit = 51

        else:
            loss_cnt += 1
            if loss_cnt > 15:
                target_emotion = IMG_NORMAL
                l_speed = r_speed = 0

        # ===============================
        # 표정 변경 시 완전 정지 + ACK 대기
        # ===============================
        if target_emotion != last_emotion_cmd:
            lcd_cmd_q.put(target_emotion)
            last_emotion_cmd = target_emotion

            if ser_motor:
                ser_motor.write(bytearray([0xFF, servo_angle, 0, 0, 0]))

            while True:
                if not lcd_ack_q.empty():
                    lcd_ack_q.get()
                    break

                if ser_motor:
                    ser_motor.write(bytearray([0xFF, servo_angle, 0, 0, 0]))

                cv2.putText(
                    frame, "Changing Face...",
                    (center_x - 80, h // 2),
                    cv2.FONT_HERSHEY_SIMPLEX, 1,
                    (0, 255, 255), 3
                )
                cv2.imshow("Robot Eye", frame)
                if cv2.waitKey(1) & 0xFF == ord("q"):
                    return

        # ---------- 모터 전송 ----------
        if ser_motor:
            packet = bytearray([0xFF, servo_angle, l_speed, r_speed, dir_bit])
            ser_motor.write(packet)

        # ---------- 디버깅 ----------
        print(
            f"\rFPS:{int(fps)} | Mode:{cmd_mode} | "
            f"Angle:{servo_angle} | Dist:{dist_score} | Owner:{is_owner}",
            end=""
        )

        cv2.imshow("Robot Eye", frame)

        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("1") and not is_owner:
            encs = face_recognition.face_encodings(rgb)
            if encs:
                is_owner = True
                print("\n🎉 주인 등록 완료!")

    # ---------- 종료 ----------
    if ser_motor:
        ser_motor.close()

    p.terminate()
    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
