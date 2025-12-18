#include <SDI12.h>
#include <math.h>

/* ============================================================
   Hardware configuration
   ============================================================ */

// SDI-12 data line (open-drain bus)
#define SDI12_DATA_PIN 2

// MOSFET gate driving the heater (PD6)
#define MOSFET_PIN 6

// Analog pins used for three thermistors
const uint8_t THERM_PINS[3] = {A0, A1, A2};

// Analog pin connected to INA180 current sense amplifier output
#define CURRENT_PIN A3

// Fixed resistor used in thermistor voltage divider (10 kΩ)
#define R_FIXED 10000.0

// Shunt resistor value used for current measurement (Ω)
#define SHUNT_RESISTOR 0.05

// INA180 gain (A1 variant = 20 V/V)
#define INA180_GAIN 20.0

// ADC reference voltage (AVcc = 5 V)
#define VREF 5.0


/* ============================================================
   SDI-12 configuration
   ============================================================ */

// Instantiate SDI-12 object
SDI12 sdi12(SDI12_DATA_PIN);

// SDI-12 sensor address
char sensorAddress = '0';


/* ============================================================
   Steinhart–Hart coefficients
   Fitted for Vishay 10K3MCD1 NTC thermistor
   ============================================================ */

const float SH_A = 0.001129098;
const float SH_B = 0.000234132;
const float SH_C = 0.0000000876505;


/* ============================================================
   Measurement storage variables
   ============================================================ */

// Temperature readings (°C) for three thermistors
float tempsC[3] = {NAN, NAN, NAN};

// Measured heater current (A)
float heaterCurrentA = NAN;

// Timing variables
unsigned long lastSampleMs = 0;
unsigned long lastCsvMs = 0;

// Measurement and logging periods
const unsigned long SAMPLE_PERIOD_MS = 1000;   // measurement update rate
const unsigned long CSV_PERIOD_MS    = 1000;   // CSV output rate


/* ============================================================
   ADC helper functions
   ============================================================ */

/*
  Perform a single ADC read with a dummy conversion first.
  The dummy read allows the ADC multiplexer capacitor to settle,
  which is critical when switching between channels with different
  source impedances (thermistors vs amplifier output).
*/
int readADC(uint8_t pin) {
  analogRead(pin);              // dummy read (discarded)
  delayMicroseconds(10);        // allow ADC sample capacitor to settle
  return analogRead(pin);       // actual measurement
}

/*
  Take multiple ADC readings and return their average.
  Averaging reduces quantization noise and electrical noise.
*/
int readADCAvg(uint8_t pin, uint8_t samples) {
  long sum = 0;
  for (uint8_t i = 0; i < samples; i++) {
    sum += readADC(pin);
  }
  return sum / samples;
}


/* ============================================================
   Thermistor measurement
   ============================================================ */

/*
  Reads a thermistor voltage divider and converts it to temperature (°C)
  using the Steinhart–Hart equation.
*/
float readThermistorC(uint8_t pin) {

  // Average 16 ADC samples for noise reduction
  int adc = readADCAvg(pin, 16);

  // Reject invalid readings (open/short conditions)
  if (adc <= 5 || adc >= 1018) return NAN;

  // Calculate thermistor resistance (low-side divider)
  float rTherm = R_FIXED * ((float)adc / (1023.0 - adc));

  // Steinhart–Hart calculation
  float lnR = log(rTherm);
  float tempK = 1.0 / (SH_A + SH_B * lnR + SH_C * lnR * lnR * lnR);

  // Convert Kelvin to Celsius
  return tempK - 273.15;
}


/* ============================================================
   Heater current measurement
   ============================================================ */

/*
  Reads the INA180 output and converts it to current (A).
*/
float readCurrentA() {

  // Average 32 ADC samples due to low signal amplitude
  int adc = readADCAvg(CURRENT_PIN, 32);

  // Reject invalid readings
  if (adc <= 2 || adc >= 1021) return NAN;

  // Convert ADC count to voltage
  float vOut = adc * (VREF / 1023.0);

  // Convert amplifier output voltage to current
  return vOut / (INA180_GAIN * SHUNT_RESISTOR);
}


/* ============================================================
   Periodic background measurements
   ============================================================ */

/*
  Updates all measurements once per second.
  Values are cached and reused for SDI-12 responses and CSV output.
*/
void updateMeasurements() {

  // Enforce fixed measurement interval
  if (millis() - lastSampleMs < SAMPLE_PERIOD_MS) return;
  lastSampleMs = millis();

  // Measure current first (low-impedance source)
  heaterCurrentA = readCurrentA();

  // Measure all thermistors
  for (uint8_t i = 0; i < 3; i++) {
    tempsC[i] = readThermistorC(THERM_PINS[i]);
  }
}


/* ============================================================
   CSV output over USB serial
   ============================================================ */

/*
  Outputs time-stamped measurements in CSV format.
  Intended for PC-side logging and analysis.
*/
void printCSV() {

  // Enforce CSV output interval
  if (millis() - lastCsvMs < CSV_PERIOD_MS) return;
  lastCsvMs = millis();

  // Time since power-up in seconds
  float time_s = millis() / 1000.0;

  // Print timestamp
  Serial.print(time_s, 3);
  Serial.print(",");

  // Print thermistor temperatures
  for (uint8_t i = 0; i < 3; i++) {
    if (isnan(tempsC[i])) Serial.print("NaN");
    else Serial.print(tempsC[i], 2);
    Serial.print(",");
  }

  // Print heater current
  if (isnan(heaterCurrentA)) Serial.println("NaN");
  else Serial.println(heaterCurrentA, 3);
}


/* ============================================================
   SDI-12 communication helpers
   ============================================================ */

/*
  Sends an SDI-12 response with required line termination.
*/
void sendResponse(String response) {
  response += "\r\n";
  delay(12);                    // SDI-12 turnaround timing
  sdi12.sendResponse(response);
}


/* ============================================================
   Setup
   ============================================================ */

void setup() {

  // Initialize serial port for CSV logging
  Serial.begin(115200);
  while (!Serial) {}

  // Enable heater (continuous ON)
  pinMode(MOSFET_PIN, OUTPUT);
  digitalWrite(MOSFET_PIN, HIGH);

  // Print CSV header once
  Serial.println("Time_s,T1,T2,T3,Current");

  // Configure analog input pins
  for (uint8_t i = 0; i < 3; i++) pinMode(THERM_PINS[i], INPUT);
  pinMode(CURRENT_PIN, INPUT);

  // Initialize SDI-12 interface
  sdi12.begin();
  sdi12.forceListen();
}


/* ============================================================
   Main loop
   ============================================================ */

void loop() {

  // Update measurements and log to CSV
  updateMeasurements();
  printCSV();

  // Handle SDI-12 commands
  if (sdi12.available()) {

    // Read SDI-12 command
    String cmd = sdi12.readStringUntil('!');
    cmd += '!';
    cmd.trim();

    // Address query
    if (cmd == "?!" || cmd == String(sensorAddress) + "?!") {
      sendResponse(String(sensorAddress));
    }

    // Address change command
    else if (cmd.length() == 4 && cmd[1] == 'A' && cmd[3] == '!') {
      if (cmd[0] == sensorAddress) {
        sensorAddress = cmd[2];
        sendResponse(String(sensorAddress));
      }
    }

    // Measurement command
    else if (cmd == String(sensorAddress) + "M!") {
      sendResponse(String(sensorAddress) + "0014");
    }

    // Data request
    else if (cmd == String(sensorAddress) + "D0!") {

      String resp = String(sensorAddress);

      // Append temperature values
      for (uint8_t i = 0; i < 3; i++) {
        if (isnan(tempsC[i])) resp += "+999.9";
        else resp += (tempsC[i] >= 0 ? "+" : "") + String(tempsC[i], 2);
      }

      // Append current value
      if (isnan(heaterCurrentA)) resp += "+999.9";
      else resp += "+" + String(heaterCurrentA, 3);

      sendResponse(resp);
    }

    // Clear buffer and resume listening
    while (sdi12.available()) sdi12.read();
    sdi12.forceListen();
  }

  delay(5);
}
