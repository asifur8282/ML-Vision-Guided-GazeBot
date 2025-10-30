import cv2
import cvzone
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import serial
import time


cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(cv2.CAP_PROP_FRAME_WIDTH, ws)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, hs)

if not cap.isOpened():
    print("❌ Camera couldn't access!")
    exit()


port = "COM7"
baud_rate = 9600 # These will vary based on the Ports the ESP32 gets connected too
ser = None

try:
    ser = serial.Serial(port, baud_rate, timeout=1)
    print(f"✅ Connected to ESP32 on {port} at {baud_rate} bps")
    time.sleep(2)  
except serial.SerialException as e:
    print(f"❌ Serial connection error: {e}")
    exit()

detector = FaceDetector()
servoPos = [90, 90]  

try:
    while True:
        ret, img = cap.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        img, bboxs = detector.findFaces(img, draw=False)

        if bboxs:
            fx, fy = bboxs[0]["center"]
            servoX = int(np.clip(np.interp(fx, [0, ws], [0, 180]), 0, 180))
            servoY = int(np.clip(np.interp(fy, [0, hs], [0, 180]), 0, 180))
            servoPos = [servoX, servoY]

            if ser and ser.is_open:
                try:
                    message = f"S{servoX:03d}{servoY:03d}\n".encode('utf-8')
                    ser.write(message)
                except serial.SerialException as e:
                    print(f"❌ Serial write error: {e}")
                    break

            cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
            cv2.putText(img, str((fx, fy)), (fx + 15, fy - 15),
                        cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
            cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
            cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)
            cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
            cv2.putText(img, "HEAD FOUND", (850, 50),
                        cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3)
        else:
            
            fx, fy = ws // 2, hs // 2
            cv2.putText(img, "NO HEAD FOUND", (880, 50),
                        cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
            cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
            cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
            cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)
            cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)

        # Show servo angle info
        cv2.putText(img, f'Servo X: {servoPos[0]} deg', (50, 50),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
        cv2.putText(img, f'Servo Y: {servoPos[1]} deg', (50, 100),
                    cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

        cv2.imshow("Face Tracker", img)

        key = cv2.waitKey(1)
        if key == ord('q'):
            print("🔚 Exiting...")
            break

except KeyboardInterrupt:
    print("🔴 Interrupted by user")

finally:
    cap.release()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()
    print("✅ Cleaned up and closed.")
