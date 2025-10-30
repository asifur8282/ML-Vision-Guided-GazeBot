import cv2
import cvzone
from cvzone.FaceDetectionModule import FaceDetector
import numpy as np
import serial  
import time

cap = cv2.VideoCapture(0)
ws, hs = 1280, 720
cap.set(3, ws)
cap.set(4, hs)

if not cap.isOpened():
    print("Camera couldn't Access!!!")
    exit()

# Serial communication setup - This will vary based on the Port the Esp32 gets connected to 
port = "COM8"  
baud_rate = 115200

ser = None  
try:
    ser = serial.Serial(port, baud_rate)
    print(f"Connected to ESP32 on {port} at {baud_rate} bps")
    time.sleep(2)  
except serial.SerialException as e:
    print(f"Error opening serial port {port}: {e}")
    exit()

detector = FaceDetector()
servoPos = [90, 90] 

while True:
    success, img = cap.read()
    img, bboxs = detector.findFaces(img, draw=False)

    if bboxs:
       
        fx, fy = bboxs[0]["center"][0], bboxs[0]["center"][1]
        pos = [fx, fy]
        #convert coordinat to servo degree
        servoX = np.interp(fx, [0, ws], [0, 180])
        servoY = np.interp(fy, [0, hs], [0, 180])

        servoX = int(np.clip(servoX, 0, 180))
        servoY = int(np.clip(servoY, 0, 180))

        servoPos[0] = servoX
        servoPos[1] = servoY

        # Send servo angles via Serial
        if ser is not None and ser.is_open:
            try:
                message = f"S{servoX:03d}{servoY:03d}\n".encode('utf-8') 
                ser.write(message)
            except serial.SerialException as e:
                print(f"Error writing to serial port: {e}")
                break

        cv2.circle(img, (fx, fy), 80, (0, 0, 255), 2)
        cv2.putText(img, str(pos), (fx+15, fy-15), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2 )
        cv2.line(img, (0, fy), (ws, fy), (0, 0, 0), 2)  # x line
        cv2.line(img, (fx, hs), (fx, 0), (0, 0, 0), 2)  # y line
        cv2.circle(img, (fx, fy), 15, (0, 0, 255), cv2.FILLED)
        cv2.putText(img, "HEAD FOUND", (850, 50), cv2.FONT_HERSHEY_PLAIN, 3, (255, 0, 255), 3 )

    else:
        cv2.putText(img, "NO HEAD FOUND", (880, 50), cv2.FONT_HERSHEY_PLAIN, 3, (0, 0, 255), 3)
        cv2.circle(img, (640, 360), 80, (0, 0, 255), 2)
        cv2.circle(img, (640, 360), 15, (0, 0, 255), cv2.FILLED)
        cv2.line(img, (0, 360), (ws, 360), (0, 0, 0), 2)  # x line
        cv2.line(img, (640, hs), (640, 0), (0, 0, 0), 2)  # y line


    cv2.putText(img, f'Servo X: {int(servoPos[0])} deg', (50, 50), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)
    cv2.putText(img, f'Servo Y: {int(servoPos[1])} deg', (50, 100), cv2.FONT_HERSHEY_PLAIN, 2, (255, 0, 0), 2)

    cv2.imshow("Image", img)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
if ser is not None and ser.is_open:
    ser.close()