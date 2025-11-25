#include <FS.h>
#include <TFT_eSPI.h>  // TFT display library
#include <SPI.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>
#include <ArduinoJson.h>
#include "secret.h"   // WiFi credentials stored here

TFT_eSPI tft = TFT_eSPI();
int8_t dotCount = 0;

#define SCREEN_WIDTH 480
#define SCREEN_HEIGHT 320

#define CENTER_X (SCREEN_WIDTH / 2)
#define CENTER_Y (SCREEN_HEIGHT / 2)

// Spinner configuration
#define RADIUS 30
#define DOT_SIZE 6
#define NUM_DOTS 12
#define SPINNER_Y (CENTER_Y + 10)

bool showScreen2 = false;  // Screen control flag
char ip_robot[20] = "###.###.###.###"; // Placeholder for robot IP
char ip_cam[20] = "###.###.###.###";   // Placeholder for camera IP

// Robot coordinates & gripper
float X = 123.4, Y = 567.8, Z = 90.1;
float Roll = 45.6, Pitch = 30.2, Gripper = 78.9;
float jointValues[6] = { 10.5, 20.3, 45.0, 90.2, 15.7, 30.6 };
char uRobotStatus = 0;
char uIP_cam[4];
char RobotStatus[20] = "IDLE";
char uPreRobotStatus = 0;

// Shared variables for ESP32-web
float esp32_X = X, esp32_Y = Y, esp32_Z = Z;
float esp32_Roll = Roll, esp32_Pitch = Pitch, esp32_Gripper = Gripper;
float esp32_jointValues[6] = { 10.5, 20.3, 45.0, 90.2, 15.7, 30.6 };
char esp32_RobotStatus[20] = "IDLE";

#define I2C_ADDRESS 0x40
WebServer server(80);
WebSocketsServer webSocket = WebSocketsServer(81);
bool webEvent = false;

// WebSocket event handler
void webSocketEvent(uint8_t num, WStype_t type, uint8_t *payload, size_t length) {
  if (type == WStype_TEXT) {
    webEvent = true;
    DynamicJsonDocument doc(1024);
    deserializeJson(doc, payload);

    if (doc.containsKey("command")) {
      String command = doc["command"];
      if (command == "getStatus") {
        DynamicJsonDocument responseDoc(1024);
        responseDoc["RobotStatus"] = esp32_RobotStatus;
        responseDoc["X"] = esp32_X;
        responseDoc["Y"] = esp32_Y;
        responseDoc["Z"] = esp32_Z;
        responseDoc["Roll"] = esp32_Roll;
        responseDoc["Pitch"] = esp32_Pitch;
        responseDoc["Gripper"] = esp32_Gripper;
        for (int i = 0; i < 6; i++) {
          responseDoc["Joint" + String(i + 1)] = esp32_jointValues[i];
        }

        String jsonString;
        serializeJson(responseDoc, jsonString);
        webSocket.sendTXT(num, jsonString);
      }
    }
    webEvent = false;
  }
}

// I2C receive event handler
void receiveEvent(int howMany) {
  if (howMany > 25) {  // Ensure full frame
    uint8_t frame[26];
    for (int i = 0; i < 26; i++) {
      frame[i] = Wire.read();
    }

    uint8_t command = frame[0];
    if (command == 0x01) {  // Robot data frame
      int16_t iX = (frame[1] << 8) | frame[2];
      int16_t iY = (frame[3] << 8) | frame[4];
      int16_t iZ = (frame[5] << 8) | frame[6];
      int16_t iRoll = (frame[7] << 8) | frame[8];
      int16_t iPitch = (frame[9] << 8) | frame[10];
      int16_t iGripper = (frame[11] << 8) | frame[12];

      int16_t iJoint[6];
      for (int i = 0; i < 6; i++) {
        iJoint[i] = (frame[13 + i * 2] << 8) | frame[14 + i * 2];
      }

      uRobotStatus = frame[25];
      switch (uRobotStatus) {
        case 0x01: strcpy(RobotStatus, "Running"); break;
        case 0x02: strcpy(RobotStatus, "Fwd Kinematic"); break;
        case 0x03: strcpy(RobotStatus, "Inv Kinematic"); break;
        default: strcpy(RobotStatus, "IDLE"); break;
      }

      // Convert to float
      X = iX / 10.
