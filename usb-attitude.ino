//
// Check data on Raspi:   screen  /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_34:B7:DA:F8:2C:D8-if00  115200
// Use port:  /dev/serial/by-id/usb-Espressif_USB_JTAG_serial_debug_unit_34:B7:DA:F8:2C:D8-if00
//

#include <Wire.h>            // ESP32
#include <Preferences.h>     // ESP32
#include <WiFi.h>            // ESP32
#include <ArduinoJson.h>     // http://librarymanager/All#ArduinoJson by Benoi Blanchon
#include <Adafruit_BNO055.h> // http://librarymanager/All#Adafruit_BNO055 by Adafruit

const int PIN_SDA = GPIO_NUM_6;
const int PIN_SCL = GPIO_NUM_7;
const int PIN_TARE = GPIO_NUM_10;
const int PIN_LED = GPIO_NUM_8;
const uint8_t BNO055_ADDR = 0x29;
const unsigned long INTERVAL_MS = 100;
const unsigned long BUTTON_MINIMAL_MS = 200;
const uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;

typedef struct {
  float roll;  /**< Rotation around the longitudinal axis (the plane body, 'X axis'). Roll is positive and increasing when moving downward. -90 degrees <= roll <= 90 degrees */
  float pitch; /**< Rotation around the lateral axis (the wing span, 'Y axis'). Pitch is positive and increasing when moving upwards. -180 degrees <= pitch <= 180 degrees) */
  float yaw;   /**< Angle between the longitudinal axis (the plane body) and magnetic north, measured clockwise when viewing from the top of the device. 0-359 degrees */
} attitude_t;

// Global variables
unsigned long lastSent = 0;
unsigned long buttonDown = 0;
Adafruit_BNO055 bno = Adafruit_BNO055(55, BNO055_ADDR, &Wire);
StaticJsonDocument<1024> doc;
Preferences preferences;
float tareYaw = 0.0;
float tareRoll = 0.0;
float tarePitch = 0.0;

void errorWait(int sec) {
  for (int i = 0; i <= sec; i++) {
    digitalWrite(PIN_LED, LOW);  // On
    delay(200);
    digitalWrite(PIN_LED, HIGH);  // Off
    delay(200);
  }
  digitalWrite(PIN_LED, LOW);  // On
}

float readFloatFromStorage(const char* key, float defaultValue) {
  preferences.begin("nvs", false);
  float value = preferences.getFloat(key, defaultValue);
  preferences.end();
  Serial.printf("# Received %s from storage: %7.03f\n", key, value);
  return value;
}

void writeFloatToStorage(const char* key, float value) {
  preferences.begin("nvs", false);
  preferences.putFloat(key, value);
  preferences.end();
  Serial.printf("# Written %s to storage: %7.03f\n", key, value);
}

void initializeJsonDocument() {
  doc["context"] = "vessels.self";
  doc["updates"][0]["$source"] = "USB.Attitude";
  doc["updates"][0]["source"]["type"] = "SERIAL";
  doc["updates"][0]["source"]["device"] = "USB Serial";
  doc["updates"][0]["source"]["label"] = "USB";
  doc["updates"][0]["source"]["src"] = "Attitude";
  doc["updates"][0]["source"]["pgn"] = 127257;
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
}

void setup() {
  Serial.begin(115200);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);  // On
  pinMode(PIN_TARE, INPUT_PULLUP);

  initializeJsonDocument();

  Serial.println();

  // Disable WiFi
  WiFi.disconnect();
  WiFi.mode(WIFI_OFF);

  tareYaw = readFloatFromStorage("tareYaw", 0.0);
  tarePitch = readFloatFromStorage("tarePitch", 0.0);
  tareRoll = readFloatFromStorage("tareRoll", 0.0);

  while (!Wire.begin(PIN_SDA, PIN_SCL)) {
    Serial.println("# Wire initialization failed. Retrying...");
    errorWait(0.4);
  }

  while (!bno.begin()) {
    Serial.println("# BNO055 not detected. Retrying...");
    errorWait(0.4);
  }
  bno.setExtCrystalUse(true);

  delay(100);
  digitalWrite(PIN_LED, HIGH);  // Off
}

float degToRad(float deg) {
  return deg * 71.0 / 4068.0;
}

bool getAttitude(attitude_t* attitude, bool applyTare = true) {
  sensors_event_t event;
  bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER);
  attitude->yaw = event.orientation.x;
  attitude->pitch = event.orientation.z;
  attitude->roll = event.orientation.y;
  if (applyTare) {
    attitude->yaw -= tareYaw;
    if (attitude->yaw < 0.0) attitude->yaw += 360.0;
    if (attitude->yaw >= 360.0) attitude->yaw -= 360.0;
    attitude->pitch -= tarePitch;
    if (attitude->pitch < -180.0) attitude->pitch += 360.0;
    if (attitude->pitch > 180.0) attitude->pitch -= 360.0;
    attitude->roll -= tareRoll;
    if (attitude->roll < -90.0) attitude->roll = -90.0 - (-90.0 - attitude->roll);  // -95 > -85
    if (attitude->roll > 90.0) attitude->roll = 90.0 + (90.0 - attitude->roll);     // 95 > 85
  }
  return true;
}

void tare() {
  attitude_t attitude;
  getAttitude(&attitude, false);
  tareYaw = attitude.yaw;
  writeFloatToStorage("tareYaw", tareYaw);
  tarePitch = attitude.pitch;
  writeFloatToStorage("tarePitch", tarePitch);
  tareRoll = attitude.roll;
  writeFloatToStorage("tareRoll", tareRoll);
}

void handleTareButton(unsigned long now) {
  bool buttonState = digitalRead(PIN_TARE);
  if (buttonState == LOW && buttonDown == 0) {
    buttonDown = now;
  }
  if (buttonState == HIGH && buttonDown && now - buttonDown > BUTTON_MINIMAL_MS) {
    tare();
    Serial.println("# TARE SET");
    buttonDown = 0;
  }
}

void sendData() {
  attitude_t attitude;
  if (getAttitude(&attitude)) {
    doc["updates"][0]["values"][0]["value"]["yaw"] = degToRad(attitude.yaw);          // + PI; // Heading in radians, increasing when turning to starboard.
    doc["updates"][0]["values"][0]["value"]["pitch"] = degToRad(attitude.pitch * -1); // Pitch in radians. Positive, when your bow rises.
    doc["updates"][0]["values"][0]["value"]["roll"] = degToRad(attitude.roll * -1);   // Roll in radians. Positive, when tilted right (starboard).
    serializeJson(doc, Serial);
    Serial.println();
  }
}

void loop() {
  unsigned long now = millis();

  handleTareButton(now);

  if (now - lastSent > INTERVAL_MS) {
    lastSent = now;
    sendData();
  }
}
