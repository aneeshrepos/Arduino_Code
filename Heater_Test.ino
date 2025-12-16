#define MOSFET_PIN 6   // PD6

void setup() {
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);   // Keep MOSFET turned ON
}

void loop() {
  // Nothing needed — MOSFET stays ON forever
}
