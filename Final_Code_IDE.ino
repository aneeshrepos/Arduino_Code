#include <SDI12.h>
#include <math.h>

/* ----------------- Hardware configuration ----------------- */
#define SDI12_DATA_PIN 2
#define MOSFET_PIN 6   // PD6
const uint8_t THERM_PINS[3] = {A0, A1, A2};
#define CURRENT_PIN A3

#define R_FIXED 10000.0
#define SHUNT_RESISTOR 0.05
#define INA180_GAIN 20.0    // INA180A1
#define VREF 5.0
/* ----------------- Heater cycle timing ----------------- */
const unsigned long BASELINE_MS = 10UL * 1000UL;
const unsigned long HEAT_MS     = 30UL * 1000UL;
const unsigned long COOL_MS     = 80UL * 1000UL;

const unsigned long TOTAL_CYCLE_MS =
  BASELINE_MS + HEAT_MS + COOL_MS;

unsigned long cycleStartMs = 0;
bool cycleActive = false;
/* ----------------- Heater control state machine ----------------- */
void updateHeaterCycle() {

  if (!cycleActive) {
    digitalWrite(MOSFET_PIN, LOW);   // heater OFF by default
    return;
  }

  unsigned long elapsed = millis() - cycleStartMs;

  if (elapsed < BASELINE_MS) {
    // Baseline phase: heater OFF
    digitalWrite(MOSFET_PIN, LOW);
  }
  else if (elapsed < BASELINE_MS + HEAT_MS) {
    // Heating phase: heater ON
    digitalWrite(MOSFET_PIN, HIGH);
  }
  else if (elapsed < TOTAL_CYCLE_MS) {
    // Cooling phase: heater OFF
    digitalWrite(MOSFET_PIN, LOW);
  }
  else {
    // Cycle complete
    digitalWrite(MOSFET_PIN, LOW);
    cycleActive = false;
  }
}

/* ----------------- SDI-12 ----------------- */
SDI12 sdi12(SDI12_DATA_PIN);
char sensorAddress = '0';

/* ----------------- Steinhart–Hart (10K3MCD1) ----------------- */
const float SH_A = 0.001129098;
const float SH_B = 0.000234132;
const float SH_C = 0.0000000876505;

/* ----------------- Measurement storage ----------------- */
float tempsC[3] = {NAN, NAN, NAN};
float heaterCurrentA = NAN;

unsigned long lastSampleMs = 0;
unsigned long lastCsvMs = 0;

const unsigned long SAMPLE_PERIOD_MS = 1000;
const unsigned long CSV_PERIOD_MS    = 1000;

/* ----------------- ADC helpers ----------------- */
int readADC(uint8_t pin) {
  analogRead(pin);              // dummy read
  delayMicroseconds(10);
  return analogRead(pin);
}

int readADCAvg(uint8_t pin, uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += readADC(pin);
  }
  return sum / samples;
}

/* ----------------- Thermistor ----------------- */
float readThermistorC(uint8_t pin) {
  int adc = readADCAvg(pin, 16);
  if (adc <= 5 || adc >= 1018) return NAN;

  float rTherm = R_FIXED * ((float)adc / (1023.0 - adc));
  float lnR = log(rTherm);
  float tempK = 1.0 / (SH_A + SH_B * lnR + SH_C * lnR * lnR * lnR);

  return tempK - 273.15;
}

/* ----------------- Current ----------------- */
float readCurrentA() {
  int adc = readADCAvg(CURRENT_PIN, 32);
  if (adc <= 2 || adc >= 1021) return NAN;

  float vOut = adc * (VREF / 1023.0);
  return vOut / (INA180_GAIN * SHUNT_RESISTOR);
}

/* ----------------- Background sampling ----------------- */
void updateMeasurements() {
  if (millis() - lastSampleMs < SAMPLE_PERIOD_MS) return;
  lastSampleMs = millis();

  heaterCurrentA = readCurrentA();

  for (uint8_t i = 0; i < 3; i++) {
    tempsC[i] = readThermistorC(THERM_PINS[i]);
  }
}

/* ----------------- CSV output over Serial ----------------- */
void printCSV() {
  if (millis() - lastCsvMs < CSV_PERIOD_MS) return;
  lastCsvMs = millis();

  float time_s = millis() / 1000.0;

  // Time column
  Serial.print(time_s, 3);
  Serial.print(",");

  // T1,T2,T3
  for (uint8_t i = 0; i < 3; i++) {
    if (isnan(tempsC[i])) Serial.print("NaN");
    else Serial.print(tempsC[i], 2);
    Serial.print(",");
  }

  // Current
  if (isnan(heaterCurrentA)) Serial.println("NaN");
  else Serial.println(heaterCurrentA, 3);
}

/* ----------------- SDI-12 response ----------------- */
void sendResponse(String response) {
  response += "\r\n";
  delay(12);
  sdi12.sendResponse(response);
}

/* ----------------- Setup ----------------- */
void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, LOW);    // Heater initially OFF
  cycleStartMs = millis();
  cycleActive = true;


  // CSV header (printed once)
  Serial.println("Time_s,T1,T2,T3,Current");

  for (uint8_t i = 0; i < 3; i++) pinMode(THERM_PINS[i], INPUT);
  pinMode(CURRENT_PIN, INPUT);

  sdi12.begin();
  sdi12.forceListen();
}

/* ----------------- Loop ----------------- */
void loop() {
  updateHeaterCycle();
  updateMeasurements();
  printCSV();

  if (sdi12.available()) {
    String cmd = sdi12.readStringUntil('!');
    cmd += '!';
    cmd.trim();

    if (cmd == "?!" || cmd == String(sensorAddress) + "?!") {
      sendResponse(String(sensorAddress));
    }

    else if (cmd.length() == 4 && cmd[1] == 'A' && cmd[3] == '!') {
      if (cmd[0] == sensorAddress) {
        sensorAddress = cmd[2];
        sendResponse(String(sensorAddress));
      }
    }

    else if (cmd == String(sensorAddress) + "M!") {
      sendResponse(String(sensorAddress) + "0014");
    }

    else if (cmd == String(sensorAddress) + "D0!") {
      String resp = String(sensorAddress);

      for (uint8_t i = 0; i < 3; i++) {
        if (isnan(tempsC[i])) resp += "+999.9";
        else resp += (tempsC[i] >= 0 ? "+" : "") + String(tempsC[i], 2);
      }

      if (isnan(heaterCurrentA)) resp += "+999.9";
      else resp += "+" + String(heaterCurrentA, 3);

      sendResponse(resp);
    }

    while (sdi12.available()) sdi12.read();
    sdi12.forceListen();
  }

  delay(5);
}
