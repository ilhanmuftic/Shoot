#include <Servo.h>

Servo shooter;
Servo rotator;

const int shooterPin = 9;   // Servo for shooting
const int rotatorPin = 10;  // Servo for rotating platform

const int shootAngle = 100; // Angle to "shoot"
const int restAngle = 0;    // Default resting angle
const int shootDelay = 500; // Time to hold shoot position in ms

String inputString = "";     // Stores incoming serial data
bool stringComplete = false; // Whether the string is complete

void setup() {
  Serial.begin(9600);

  shooter.attach(shooterPin);
  rotator.attach(rotatorPin);

  shooter.write(restAngle);
  rotator.write(90); // Start at center (optional)

  inputString.reserve(10); // Reserve some memory
}

void loop() {
  if (stringComplete) {
    processCommand(inputString);
    inputString = "";
    stringComplete = false;
  }
}

// Read serial input one character at a time
void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();

    if (inChar == '\n') {
      stringComplete = true;
    } else {
      inputString += inChar;
    }
  }
}

// Parse and handle command
void processCommand(String cmd) {
  cmd.trim(); // Remove whitespace

  if (cmd == "S") {
    Serial.println("Shoot command received");

    shooter.write(shootAngle);
    delay(shootDelay);
    shooter.write(restAngle);
  } else if (cmd.startsWith("R")) {
    int angle = cmd.substring(1).toInt();

    if (angle >= 0 && angle <= 180) {
      rotator.write(angle);
      Serial.print("Rotating to ");
      Serial.println(angle);
    } else {
      Serial.println("Invalid angle (0–180 only)");
    }
  } else {
    Serial.println("Unknown command");
  }
}
