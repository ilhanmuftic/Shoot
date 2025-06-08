import cv2
import face_recognition
import numpy as np
import time
import serial
import threading
import speech_recognition as sr

# === CONFIGURATION ===
SERIAL_PORT = '/dev/ttyUSB0'
COOLDOWN_SECONDS = 3

# === SETUP SERIAL ===
try:
    ser = serial.Serial(SERIAL_PORT, 9600, timeout=1)
    time.sleep(2)
except serial.SerialException:
    print(f"Error: Could not open serial port {SERIAL_PORT}")
    ser = None

# === SHOOT FUNCTION ===
def shoot():
    if ser and ser.is_open:
        ser.write(b'S\n')
        print("💥 SHOOT command sent to Arduino")
    else:
        print("Serial port not available, can't shoot")

# === ROTATE FUNCTION ===
def send_rotation(angle):
    angle = max(0, min(180, angle))
    command = f"R{angle}\n"
    if ser and ser.is_open:
        ser.write(command.encode())
        print(f"🔁 Sent rotation command: {command.strip()}")
    else:
        print("Serial port not available, can't rotate")

# === VOICE CONTROL ===
shoot_command_flag = False

def listen_for_shoot():
    global shoot_command_flag
    recognizer = sr.Recognizer()
    mic = sr.Microphone()

    with mic as source:
        recognizer.adjust_for_ambient_noise(source)

    print("🎙️ Voice control ready. Say 'shoot' to fire.")

    while True:
        with mic as source:
            try:
                audio = recognizer.listen(source, timeout=5)
                command = recognizer.recognize_google(audio).lower()
                print(f"🗣️ Heard: {command}")
                if "shoot" in command:
                    shoot_command_flag = True
            except sr.WaitTimeoutError:
                continue
            except sr.UnknownValueError:
                print("🤔 Didn't catch that.")
            except sr.RequestError as e:
                print(f"Speech recognition error: {e}")

# === MAIN ===
def main():
    global shoot_command_flag

    # Load and encode known face
    friend_image = face_recognition.load_image_file("friend.jpg")
    friend_face_encodings = face_recognition.face_encodings(friend_image)
    if not friend_face_encodings:
        print("❌ No face found in friend.jpg!")
        return
    friend_face_encoding = friend_face_encodings[0]

    # Setup video
    video_capture = cv2.VideoCapture(0)
    if not video_capture.isOpened():
        print("❌ Could not open webcam")
        return

    frame_width = int(video_capture.get(cv2.CAP_PROP_FRAME_WIDTH))
    frame_center_x = frame_width // 2

    # Start voice listener thread
    threading.Thread(target=listen_for_shoot, daemon=True).start()

    print("📹 Starting turret. Press 'q' to quit.")

    while True:
        ret, frame = video_capture.read()
        if not ret:
            print("❌ Failed to grab frame")
            break

        rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        face_locations = face_recognition.face_locations(rgb_frame)
        face_encodings = face_recognition.face_encodings(rgb_frame, face_locations)

        for (top, right, bottom, left), face_encoding in zip(face_locations, face_encodings):
            matches = face_recognition.compare_faces([friend_face_encoding], face_encoding, tolerance=0.5)
            is_friend = matches[0]

            face_center_x = (left + right) // 2

            if is_friend:
                cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)
                cv2.putText(frame, "Target", (left, top - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 0, 255), 2)

                # Rotation
                offset_x = face_center_x - frame_center_x
                normalized = offset_x / (frame_width // 2)
                angle = int(90 + normalized * 45)
                send_rotation(angle)

                if abs(offset_x) < 30 and shoot_command_flag:
                    shoot()
                    shoot_command_flag = False

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
