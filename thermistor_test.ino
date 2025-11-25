const int thermistorPin = A0;
const float R_FIXED = 10000.0;  // your series resistor
const float BETA = 3950.0;
const float R0 = 10000.0;
const float T0 = 298.15;

void setup() {
  Serial.begin(9600);
  Serial.println("Thermistor temperature test");
}

void loop() {
  int adc = analogRead(thermistorPin);
  float vRatio = adc / 1023.0;

  // === pick one of these two ===
  //float R_T = R_FIXED * (1 / vRatio - 1);        // if thermistor on bottom
  float R_T = R_FIXED * (vRatio / (1 - vRatio));   // if thermistor on top

  float tempK = 1 / ((1 / T0) + (1 / BETA) * log(R_T / R0));
  float tempC = tempK - 273.15;

  Serial.print("ADC=");
  Serial.print(adc);
  Serial.print("  R_T=");
  Serial.print(R_T, 1);
  Serial.print(" ohms  Temp=");
  Serial.print(tempC, 2);
  Serial.println(" C");
  delay(1000);
}