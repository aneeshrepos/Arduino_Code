#define MOSFET_PIN 6     // PD6
#define CURRENT_PIN A3  // PC3 / ADC3 (INA180 OUT)

#define SHUNT_RESISTOR 0.05   // ohms
#define INA180_GAIN    50.0   // <-- CHANGE if not A2
#define VREF           5.0    // ADC reference

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);   // Turn heater ON

  pinMode(CURRENT_PIN, INPUT);

  Serial.println("Heater ON - Current Sense Active");
}

float readCurrentA() {
  // Dummy read for ADC mux settling
  analogRead(CURRENT_PIN);
  delayMicroseconds(5);

  int adc = analogRead(CURRENT_PIN);
  if (adc <= 0 || adc >= 1023) return NAN;

  float vOut = adc * (VREF / 1023.0);

  // I = Vout / (Gain * Rshunt)
  float current = vOut / (INA180_GAIN * SHUNT_RESISTOR);
  return current;
}

void loop() {
  float currentA = readCurrentA();

  Serial.print("ADC: ");
  Serial.print(analogRead(CURRENT_PIN));

  Serial.print(" | Current: ");
  if (isnan(currentA)) {
    Serial.println("ERR");
  } else {
    Serial.print(currentA, 3);
    Serial.println(" A");
  }

  delay(500);
}
