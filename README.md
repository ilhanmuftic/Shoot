# 🎯 Face Recognition BB Gun Turret

An automated turret system that uses face recognition to detect a specific person ("friend") and activate a **real BB gun** via an Arduino-controlled trigger mechanism.

> ⚠️ This is a **serious robotics + weapon project**. Please read the safety and legal disclaimers below before continuing.

---

## 🛡️ Features

- Real-time face detection using your webcam
- Facial recognition of a predefined target (from `friend.jpg`)
- Automated firing via Arduino and servo trigger
- Cooldown logic to prevent rapid firing
- Visual labels and bounding boxes in OpenCV preview

---

## ⚠️ Safety & Legal Notice

> 🔥 This system controls an actual BB gun. It is **potentially dangerous** and may be **illegal** in your country or region.

- Never aim this system at **people**, **animals**, or **public spaces**
- Use only in a **secure, enclosed test environment**
- **Comply with all local laws** related to automated firearms or air-powered weapons
- Add manual hardware safeties (switches, locks) before operation
- Replace the BB gun with a dummy or LED during initial development
- The creator(s) take **no responsibility** for misuse, injury, or legal consequences

---

## 🧰 Hardware Required

- Arduino Uno/Nano (or compatible)
- BB Gun with trigger mechanism actuated via servo
- Servo motor (e.g., MG996R or similar torque)
- External 5V power supply for servo (2A+)
- Jumper wires, breadboard, or soldered circuit
- Webcam connected to PC

---

## 🧠 How It Works

1. Load an image of your “friend” as `friend.jpg`
2. Python script uses OpenCV + `face_recognition` to detect faces in webcam feed
3. If a match is found, it sends an `'S'` command to the Arduino over serial
4. Arduino moves a servo to pull the BB gun trigger and then resets

---

## 💻 Software Setup (Python Side)

### Dependencies

Install required Python libraries:

```bash
pip install face_recognition opencv-python numpy pyserial
