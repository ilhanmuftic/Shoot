#include <Servo.h>

Servo shooter;
const int shooterPin = 9;   // Connect your servo signal wire to pin 9
const int shootAngle = 100; // Angle to "shoot"
const int restAngle = 0;    // Default resting angle
const int shootDelay = 500; // Time to hold shoot position in ms

void setup()
{
  Serial.begin(9600);
  shooter.attach(shooterPin);
  shooter.write(restAngle);
}

void loop()
{
  if (Serial.available() > 0)
  {
    char command = Serial.read();

    if (command == 'S')
    {
      Serial.println("Shoot command received");

      // Perform shoot action
      shooter.write(shootAngle);
      delay(shootDelay);
      shooter.write(restAngle);
    }
  }
}
