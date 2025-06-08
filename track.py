import cv2
import face_recognition
import numpy as np
import time
import serial

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyUSB0'
FRAME_WIDTH = 640  # Can be set manually or fetched from capture
COOLDOWN_SECONDS = 3

# === SETUP SERIAL ===
try:
    ser = serial.Serial(SERIAL_PORT, 9600, timeout=1)
    time.sleep(2)  # wait for Arduino reset after serial connection opens
except serial.SerialException:
    print(f"Error: Could not open serial port {SERIAL_PORT}")
    ser = None

# === SHOOT FUNCTION ===
def shoot():
    if ser and ser.is_open:
        ser.write(b'S\n')  # send 'S' with newline for Arduino parser
        print("SHOOT command sent to Arduino")
    else:
        print("Serial port not available, can't shoot")

# === ROTATE FUNCTION ===
def send_rotation(angle):
    angle = max(0, min(180, angle))  # constrain to 0–180
    command = f"R{angle}\n"
    if ser and ser.is_open:
        ser.write(command.encode())
        print(f"Sent rotation command: {command.strip()}")
    else:
        print("Serial port not available, can't rotate")

# === MAIN ===
def main():
    # Load and encode known face
    friend_image = face_recognition.load_image_file("friend.jpg")
    friend_face_encodings = face_recognition.face_encodings(friend_image)
    if not friend_face_encodings:
        print("No face found in friend.jpg!")
        return
    friend_face_encoding = friend_face_encodings[0]

    # Setup video
    video_capture = cv2.VideoCapture(0)
    if not video_capture.isOpened():
        print("Error: Could not open webcam")
        return

    frame_width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_center_x = frame_width // 2

    print("Starting video stream. Press 'q' to quit.")

    last_shot_time = 0

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

            face_center_x = (left + right) // 2

            if is_friend:
                friend_detected = True
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                cv2.putText(frame, "Target", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                # === ROTATE LOGIC ===
                offset_x = face_center_x - frame_center_x
                normalized = offset_x / (frame_width // 2)  # -1 to 1
                angle = int(90 + normalized * 45)  # Map to 45–135 range
                send_rotation(angle)

                print(f"Friend at X={face_center_x}, angle={angle}")

                current_time = time.time()
                if abs(offset_x) < 30 and (current_time - last_shot_time) > COOLDOWN_SECONDS:
                    shoot()
                    last_shot_time = current_time

            else:
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 255, 0), 2)
                cv2.putText(frame, "Civilian", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

        cv2.imshow('Face Recognition Turret', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    video_capture.release()
    cv2.destroyAllWindows()
    if ser and ser.is_open:
        ser.close()

if __name__ == "__main__":
    main()
