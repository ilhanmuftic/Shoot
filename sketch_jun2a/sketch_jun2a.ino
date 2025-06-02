const int shootPin = 9;  // Example pin to trigger turret shooting (e.g., a relay or servo)

void setup() {
  pinMode(shootPin, OUTPUT);
  Serial.begin(9600);
  digitalWrite(shootPin, LOW);
  Serial.println("Arduino ready, waiting for command...");
}

void loop() {
  if (Serial.available()) {
    char cmd = Serial.read();
    Serial.print("Received command: ");
    Serial.println(cmd);

    if (cmd == 'S') {
      Serial.println("Shooting!");
      shoot();
      Serial.println("Done shooting.");
    } else {
      Serial.println("Unknown command.");
    }
  }
}

void shoot() {
  digitalWrite(shootPin, HIGH);  // Activate shoot pin
  delay(500);                    // Shoot duration (adjust as needed)
  digitalWrite(shootPin, LOW);
}
