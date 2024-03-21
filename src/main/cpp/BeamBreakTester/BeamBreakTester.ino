// BEAM BREAK TESTER

bool lastState = false;

void setup() {
  Serial.begin(9600);
  pinMode(10, INPUT_PULLUP);
  pinMode(11, OUTPUT);

  lastState = !digitalRead(10);
}

void loop() {
  bool currentState = digitalRead(10);  

  if (lastState != currentState) {
    digitalWrite(11, currentState);
    lastState = currentState;
    Serial.print("Beam Break: ");

    if (currentState) {
      Serial.println("Broken");
    } else {
      Serial.println("Unbroken");
    }
  }

  delay(250);
}