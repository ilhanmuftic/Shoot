# Face Recognition Turret

A DIY smart turret that detects a specific target face using a webcam, automatically rotates to track it, and "shoots" on command via serial communication with an Arduino-controlled servo system. It supports commands via face tracking, serial, and voice control.

---

## Features

- Face recognition-based targeting using `face_recognition` and OpenCV.
- Servo motor control for rotation and shooting via Arduino.
- Voice control to trigger shooting.
- Python scripts to handle face detection, rotation, and shooting commands.
- Real-time video display with target highlighting.
- Cooldown timer to prevent rapid firing.

---

## Hardware Requirements

- Arduino Uno (or compatible)
- Two Servo motors
  - One for rotating the turret platform
  - One for triggering the shooting mechanism
- USB cable for serial communication between Arduino and PC
- Webcam

---

## Software Requirements

- Arduino IDE
- Python 3.7+
- Python libraries:
  - `opencv-python`
  - `face_recognition`
  - `numpy`
  - `pyserial`
  - `speechrecognition`
  - `pyaudio` (or other microphone drivers)

---

## Arduino Code

Upload the provided Arduino sketch to your Arduino board. It listens on serial for two commands:

- `S` — Shoot (moves servo to shoot angle and back)
- `R<angle>` — Rotate (move servo to specified angle between 0 and 180)

Pins used:

- Pin 9: Shooter servo
- Pin 10: Rotator servo

---

## Python Scripts

### track.py

- Tracks a known face (`friend.jpg`)
- Rotates turret platform to center on the face
- Shoots automatically when target is centered (within 30 pixels)
- Cooldown to avoid repeated shooting

### turret.py

- Simplified script to detect friend face and shoot on cooldown

### voice.py

- Adds voice control using speech recognition
- Say "shoot" to fire when the target is centered

---

## Usage

1. Connect Arduino and servos as per pin configuration.
2. Upload Arduino sketch.
3. Install Python dependencies:

   ```bash
   pip install opencv-python face_recognition numpy pyserial speechrecognition pyaudio

4. Place friend.jpg with the face of the target you want to track.

5. Run one of the Python scripts:

python track.py

python turret.py

python voice.py

6. Use the webcam window to monitor detection.

7. For voice.py, speak "shoot" to trigger the shooter.


