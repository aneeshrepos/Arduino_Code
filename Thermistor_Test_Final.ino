#include <Arduino.h>
#include <math.h>

#define R_FIXED 10000.0   // 10k fixed resistor

// Analog pins
const uint8_t thermPins[3] = {A0, A1, A2};

// Steinhart–Hart coefficients fitted for 10K3MCD1
const float A = 0.001129098;
const float B = 0.000234132;
const float C = 0.0000000876505;

// Read one thermistor and return temperature in °C
float readThermistorC(uint8_t pin) {
  int adc = analogRead(pin);

  // Fault detection
  if (adc <= 0 || adc >= 1023) {
    return NAN;
  }

  // Low-side NTC divider math
  float rTherm = R_FIXED * ((float)adc / (1023.0 - adc));

  float lnR = log(rTherm);
  float tempK = 1.0 / (A + B * lnR + C * lnR * lnR * lnR);

  return tempK - 273.15;
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("3x NTC Thermistor Readout (10K3MCD1)");
}

void loop() {
  float t0 = readThermistorC(A0);
  float t1 = readThermistorC(A1);
  float t2 = readThermistorC(A2);

  Serial.print("T0: ");
  if (isnan(t0)) Serial.print("ERR");
  else Serial.print(t0, 2);
  Serial.print(" C | ");

  Serial.print("T1: ");
  if (isnan(t1)) Serial.print("ERR");
  else Serial.print(t1, 2);
  Serial.print(" C | ");

  Serial.print("T2: ");
  if (isnan(t2)) Serial.print("ERR");
  else Serial.print(t2, 2);
  Serial.println(" C");

  delay(1000);
}
