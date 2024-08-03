
#include <Wire.h>
#include <ArduinoJson.h>                      // http://librarymanager/All#ArduinoJson by Benoi Blanchon
#include <SparkFun_BNO08x_Arduino_Library.h>  // http://librarymanager/All#SparkFun_BNO08x by SparkFun

#define BNO08X_INT GPIO_NUM_3
#define BNO08X_RST GPIO_NUM_4
#define BNO08X_ADDR 0x4B
#define PIN_SDA GPIO_NUM_6
#define PIN_SCL GPIO_NUM_7
#define PIN_TARE GPIO_NUM_10
#define PIN_LED GPIO_NUM_8
#define INTERVAL_MS 500
#define BUTTON_MINIMAL_MS 200

// Global variables
unsigned long lastSent = 0;
unsigned long buttonDown = 0;
BNO08x myIMU;
JsonDocument doc;

void errorWait(int sec) {
  for (int i = 0; i <= sec; i++) {
    digitalWrite(PIN_LED, LOW);  // On
    delay(200);
    digitalWrite(PIN_LED, HIGH);  // Off
    delay(200);
  }
  digitalWrite(PIN_LED, LOW);  // On
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);  // On
  pinMode(PIN_TARE, INPUT_PULLUP);

  doc["updates"][0]["source"]["type"] = "SERIAL";
  doc["updates"][0]["source"]["device"] = "USB Serial";
  doc["updates"][0]["source"]["label"] = "USB";
  doc["updates"][0]["source"]["src"] = "USB-Attitude";
  doc["updates"][0]["source"]["pgn"] = 127257;
  doc["updates"][0]["timestamp"] = 0.001; // float
  doc["updates"][0]["values"][0]["path"] = "navigation.attitude";
  doc["updates"][0]["values"][0]["value"]["yaw"] = 0;
  doc["updates"][0]["values"][0]["value"]["pitch"] = 0;
  doc["updates"][0]["values"][0]["value"]["roll"] = 0;
  doc["updates"][0]["meta"][0]["path"] = "navigation.attitude.yaw";
  doc["updates"][0]["meta"][0]["value"]["units"] = "rad";
  doc["updates"][0]["meta"][0]["value"]["description"] = "Yaw, +ve is heading change to starboard";
  doc["updates"][0]["meta"][1]["path"] = "navigation.attitude.pitch";
  doc["updates"][0]["meta"][1]["value"]["units"] = "rad";
  doc["updates"][0]["meta"][1]["value"]["description"] = "Pitch, +ve is bow up";
  doc["updates"][0]["meta"][2]["path"] = "navigation.attitude.roll";
  doc["updates"][0]["meta"][2]["value"]["units"] = "rad";
  doc["updates"][0]["meta"][2]["value"]["description"] = "Vessel roll, +ve is list to starboard";

  // doc["context"] = "self";
  // doc["$source"] = "USB.Attitude";
  // doc["meta"]["description"] = "Yaw Pitch Roll";
  // doc["meta"]["properties"]["roll"]["type"] = "number";
  // doc["meta"]["properties"]["roll"]["description"] = "Vessel roll, +ve is list to starboard";
  // doc["meta"]["properties"]["roll"]["units"] = "rad";
  // doc["meta"]["properties"]["pitch"]["type"] = "number";
  // doc["meta"]["properties"]["pitch"]["description"] = "Pitch, +ve is bow up";
  // doc["meta"]["properties"]["pitch"]["units"] = "rad";
  // doc["meta"]["properties"]["yaw"]["type"] = "number";
  // doc["meta"]["properties"]["yaw"]["description"] = "Yaw, +ve is heading change to starboard";
  // doc["meta"]["properties"]["yaw"]["units"] = "rad";
  // doc["value"]["yaw"] = 0;
  // doc["value"]["pitch"] = 0;
  // doc["value"]["roll"] = 0;
  // doc["timestamp"] = 0;
  // doc["pgn"] = 127257;

  while (!Serial) {
    errorWait(1);  // Wait for Serial to become available.
  }

  Serial.println();

  Wire.begin(PIN_SDA, PIN_SCL);

  while (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    Serial.println("# BNO08x not detected at default I2C address. Retrying...");
    errorWait(1);
  }
  setReports();

  delay(100);
  digitalWrite(PIN_LED, HIGH);  // Off
}

void setReports(void) {
  if (!myIMU.enableRotationVector()) {
    Serial.println("# Could not enable rotation vector");
  }
}

void loop() {
  unsigned long now = millis();

  //// Detect IMU Reset
  if (myIMU.wasReset()) {
    Serial.println("# BNO08x was reset.");
    setReports();
  }

  //// Tare Button
  bool buttonState = digitalRead(PIN_TARE);
  if (buttonState == LOW && buttonDown == 0) {
    buttonDown = now;
  }
  if (buttonState == HIGH && buttonDown && now - buttonDown > BUTTON_MINIMAL_MS) {
    myIMU.tareNow();
    myIMU.saveTare();
    Serial.println("# TARE SET");
    buttonDown = 0;
  }

  // Send data at interval
  if (now - lastSent > INTERVAL_MS) {
    lastSent = now;
    if (myIMU.getSensorEvent() == true && myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR) {
      double yaw = myIMU.getYaw();         // + PI;   // Heading in radians.
      double roll = myIMU.getRoll() * -1;  // Roll in radians. Positive, when tilted right (starboard).
      double pitch = myIMU.getPitch();     // Pitch in radians. Positive, when your bow rises.

      doc["updates"][0]["timestamp"] = float(now / 1000);
      doc["updates"][0]["values"][0]["value"]["yaw"] = yaw;
      doc["updates"][0]["values"][0]["value"]["pitch"] = pitch;
      doc["updates"][0]["values"][0]["value"]["roll"] = roll;

//       doc["value"]["yaw"] = yaw;
//       doc["value"]["pitch"] = pitch;
//       doc["value"]["roll"] = roll;
//       doc["timestamp"] = now / 1000;

      serializeJson(doc, Serial);
      Serial.println();
    }
  }
}
