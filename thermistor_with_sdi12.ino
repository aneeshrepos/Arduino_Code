#include <SDI12.h>

#define SDI12_DATA_PIN 4   // SDI-12 bus pin
#define THERM_PIN A0       // Thermistor voltage divider output
#define R_FIXED 10000.0    // Fixed resistor value (10kΩ)

SDI12 sdi12(SDI12_DATA_PIN);

char sensorAddress = '0';  // Sensor SDI-12 address
bool measurementReady = false;
float lastTemperature = 0.0;
unsigned long measurementTimestamp = 0;

// ---- Steinhart-Hart coefficients for common 10k NTC thermistor ----
const float A = 0.001129148;
const float B = 0.000234125;
const float C = 0.0000000876741;

// ---- Function to compute temperature in °C ----
float readTemperatureC() {
  int adc = analogRead(THERM_PIN);
  float v = adc * (5.0 / 1023.0);
  if (v <= 0.01) return NAN;

  float rTherm = R_FIXED * (5.0 / v - 1.0);
  float lnR = log(rTherm);
  float tempK = 1.0 / (A + B * lnR + C * pow(lnR, 3));
  return tempK - 273.15;
}

// ---- Helper for sending SDI-12 response ----
void sendResponse(String response) {
  response += "\r\n";
  Serial.print("Sending response: ");
  Serial.print(response);
  delay(12);
  sdi12.sendResponse(response);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("SDI-12 Thermistor Sensor (Reports temperature)");

  pinMode(THERM_PIN, INPUT);
  sdi12.begin();
  sdi12.forceListen();
  delay(200);
}

void loop() {
  if (sdi12.available()) {
    String cmd = sdi12.readStringUntil('!');
    cmd += '!';
    cmd.trim();

    Serial.print("Received command: ");
    Serial.println(cmd);

    // ---- Address query ----
    if (cmd == "?!" || cmd == String(sensorAddress) + "?!") {
      sendResponse(String(sensorAddress));
    }
    // ---- Change address (aAb!) ----
    else if (cmd.length() == 4 && cmd[1] == 'A' && cmd[3] == '!') {
      char currentAddr = cmd[0];
      char newAddr = cmd[2];
      if (currentAddr == sensorAddress) {
        sensorAddress = newAddr;
        sendResponse(String(newAddr));
      }
    }
    // ---- Measurement request (aM!) ----
    else if (cmd == String(sensorAddress) + "M!") {
      Serial.println("Measurement command received");

      // Respond with wait time (001) and number of values (1)
      sendResponse(String(sensorAddress) + "0011");

      measurementTimestamp = millis();
      measurementReady = true;
    }
    // ---- Data request (aD0!) ----
    else if (cmd == String(sensorAddress) + "D0!" && measurementReady) {
      Serial.println("Sending temperature data");

      lastTemperature = readTemperatureC();

      String dataResponse = String(sensorAddress);
      if (isnan(lastTemperature)) {
        dataResponse += "+999.9";  // Error value
      } else {
        dataResponse += (lastTemperature >= 0 ? "+" : "") + String(lastTemperature, 2);
      }

      sendResponse(dataResponse);
      measurementReady = false;
    }

    // ---- Clear buffer and resume listening ----
    while (sdi12.available()) sdi12.read();
    sdi12.forceListen();
  }

  delay(5);
}
