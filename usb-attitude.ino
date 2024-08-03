//
// Check data on Raspi:   screen  /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_34:B7:DA:F8:2C:D8-if00  115200
// Use port:  /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_34:B7:DA:F8:2C:D8-if00
//

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
#define INTERVAL_MS 200
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
  doc["context"] = "vessels.self";
  doc["updates"][0]["$source"] = "USB.Attitude";
  doc["updates"][0]["source"]["type"] = "SERIAL";
  doc["updates"][0]["source"]["device"] = "USB Serial";
  doc["updates"][0]["source"]["label"] = "USB";
  doc["updates"][0]["source"]["src"] = "Attitude";
  doc["updates"][0]["source"]["pgn"] = 127257;
  // doc["updates"][0]["timestamp"] = 0.0; // float
  doc["updates"][0]["values"][0]["path"] = "navigation.attitude";
  doc["updates"][0]["values"][0]["value"]["yaw"] = 0.0;
  doc["updates"][0]["values"][0]["value"]["pitch"] = 0.0;
  doc["updates"][0]["values"][0]["value"]["roll"] = 0.0;
  doc["updates"][0]["meta"][0]["path"] = "navigation.attitude.yaw";
  doc["updates"][0]["meta"][0]["value"]["units"] = "rad";
  doc["updates"][0]["meta"][0]["value"]["description"] = "Yaw, +ve is heading change to starboard";
  doc["updates"][0]["meta"][1]["path"] = "navigation.attitude.pitch";
  doc["updates"][0]["meta"][1]["value"]["units"] = "rad";
  doc["updates"][0]["meta"][1]["value"]["description"] = "Pitch, +ve is bow up";
  doc["updates"][0]["meta"][2]["path"] = "navigation.attitude.roll";
  doc["updates"][0]["meta"][2]["value"]["units"] = "rad";
  doc["updates"][0]["meta"][2]["value"]["description"] = "Vessel roll, +ve is list to starboard";

  while (!Serial) {
    errorWait(0.4);  // Wait for Serial to become available.
  }

  Serial.println();

  Wire.begin(PIN_SDA, PIN_SCL);

  while (myIMU.begin(BNO08X_ADDR, Wire, BNO08X_INT, BNO08X_RST) == false) {
    // Serial.println("# BNO08x not detected at default I2C address. Retrying...");
    errorWait(0.4);
  }
  setReports();

  delay(100);
  digitalWrite(PIN_LED, HIGH);  // Off
}

void setReports(void) {
  while (!myIMU.enableRotationVector()) {
  // while (!myIMU.enableGeomagneticRotationVector()) {
    // Serial.println("# Could not enable rotation vector. Retrying...");
    errorWait(0.4);
  }
}

void loop() {
  unsigned long now = millis();

  //// Detect IMU Reset
  if (myIMU.wasReset()) {
    // Serial.println("# BNO08x was reset.");
    setReports();
  }

  //// Tare Button
  bool buttonState = digitalRead(PIN_TARE);
  if (buttonState == LOW && buttonDown == 0) {
    buttonDown = now;
  }
  if (buttonState == HIGH && buttonDown && now - buttonDown > BUTTON_MINIMAL_MS) {
    //myIMU.clearTare();
    //myIMU.saveTare();
    myIMU.tareNow(false, SH2_TARE_BASIS_ROTATION_VECTOR);
    myIMU.saveTare();
    // Serial.println("# TARE SET");
    buttonDown = 0;
  }

  // Send data at interval
  if (now - lastSent > INTERVAL_MS) {
    lastSent = now;
    if (myIMU.getSensorEvent() == true && (myIMU.getSensorEventID() == SENSOR_REPORTID_ROTATION_VECTOR || myIMU.getSensorEventID() == SENSOR_REPORTID_GEOMAGNETIC_ROTATION_VECTOR)) {

      doc["updates"][0]["values"][0]["value"]["yaw"] = myIMU.getYaw();          // + PI; // Heading in radians, increasing when turning to starboard.
      doc["updates"][0]["values"][0]["value"]["pitch"] = myIMU.getRoll() * -1;  // Roll in radians. Positive, when tilted right (starboard).
      doc["updates"][0]["values"][0]["value"]["roll"] = myIMU.getPitch();       // Pitch in radians. Positive, when your bow rises.

      serializeJson(doc, Serial);
      Serial.println();
    }
  }
}
