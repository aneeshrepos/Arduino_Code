#include <SDI12.h>

#define SDI12_DATA_PIN 2  // SDI-12 bus pin
SDI12 sdi12(SDI12_DATA_PIN);

char sensorAddress = '0';  // Current sensor address
bool measurementReady = false;
unsigned long measurementTimestamp = 0;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  Serial.println("SDI-12 Mock Sensor (Reports ms uptime)");

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

    // Handle address query
    if (cmd == "?!" || cmd == String(sensorAddress) + "?!") {
      sendResponse(String(sensorAddress));
    }
    // Handle change address: aAb!
    else if (cmd.length() == 4 && cmd[1] == 'A' && cmd[3] == '!') {
      char currentAddr = cmd[0];
      char newAddr = cmd[2];
      if (currentAddr == sensorAddress) {
        sensorAddress = newAddr;
        sendResponse(String(newAddr));
      }
    }
    // Handle measurement: aM!
    else if (cmd == String(sensorAddress) + "M!") {
      Serial.println("Measurement command received");

      // Respond with wait time (1 sec) and number of values (1)
      sendResponse(String(sensorAddress) + "0011");

      // Record timestamp and flag for data availability
      measurementTimestamp = millis();  
      measurementReady = true;
    }
    // Handle data request: aD0!
    else if (cmd == String(sensorAddress) + "D0!" && measurementReady) {
      Serial.println("Sending measurement data");

      // Calculate milliseconds elapsed since startup
      unsigned long uptime = measurementTimestamp;  // Use the time when command was sent

      // Prepare SDI-12 data response
      String dataResponse = String(sensorAddress) + "+";
      dataResponse += String(uptime);

      sendResponse(dataResponse);

      measurementReady = false;
    }

    // Clear buffer and listen
    while (sdi12.available()) sdi12.read();
    sdi12.forceListen();
  }

  delay(5);
}

// Helper for standard SDI-12 response
void sendResponse(String response) {
  response += "\r\n";
  Serial.print("Sending response: ");
  Serial.print(response);

  delay(12);
  sdi12.sendResponse(response);
}
