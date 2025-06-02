import cv2
import face_recognition
import numpy as np
import time
import serial

# Initialize serial connection to Arduino
try:
    ser = serial.Serial('/dev/ttyUSB0', 9600, timeout=1)
    time.sleep(2)  # wait for Arduino reset after serial connection opens
except serial.SerialException:
    print("Error: Could not open serial port /dev/ttyUSB0")
    ser = None

def shoot():
    if ser and ser.is_open:
        ser.write(b'S')  # send 'S' command to Arduino
        print("SHOOT command sent to Arduino")
    else:
        print("Serial port not available, can't shoot")

def main():
    friend_image = face_recognition.load_image_file("friend.jpg")
    friend_face_encodings = face_recognition.face_encodings(friend_image)
    
    if len(friend_face_encodings) == 0:
        print("No face found in friend.jpg!")
        return
    friend_face_encoding = friend_face_encodings[0]

    video_capture = cv2.VideoCapture(0)
    if not video_capture.isOpened():
        print("Error: Could not open webcam")
        return

    print("Starting video stream. Press 'q' to quit.")

    last_shot_time = 0
    cooldown_seconds = 3

    while True:
        ret, frame = video_capture.read()
        if not ret:
            print("Failed to grab frame")
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        friend_detected = False

        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            matches = face_recognition.compare_faces([friend_face_encoding], face_encoding, tolerance=0.5)
            is_friend = matches[0]

            if is_friend:
                friend_detected = True
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                cv2.putText(frame, "Friend", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

                face_center_x = (left + right) // 2
                face_center_y = (top + bottom) // 2
                print(f"Friend detected at ({face_center_x}, {face_center_y})")

            else:
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                cv2.putText(frame, "Unknown", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

        current_time = time.time()
        if friend_detected and (current_time - last_shot_time) > cooldown_seconds:
            shoot()
            last_shot_time = current_time

        cv2.imshow('Face Recognition Turret', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()

    if ser and ser.is_open:
        ser.close()

if __name__ == "__main__":
    main()
