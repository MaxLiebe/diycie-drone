// SPDX-License-Identifier: LGPL-3.0-or-later
// Copyright 2016-2025 Hristo Gochkov, Mathieu Carbou, Emil Muratov

//
// Show how to log the incoming request and response as a curl-like syntax
//

#include <Arduino.h>
#include "AsyncUDP.h"
#include <AsyncTCP.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>

//Make sure to install the ArduinoJson library!
#include <ArduinoJson.h>
#include <AsyncJson.h>
#include <AsyncMessagePack.h>

static AsyncWebServer server(3000);
AsyncUDP udp;

#define ssid "DroneKen"
#define password "ufogobrr"

int battery = 0;
int8_t x_value;
int8_t y_value;
int8_t s_value;
int8_t l_value;

void parseControllerInput(String input) {
  int xPos = 0;
  int yPos = 5;
  int sPos = 10;
  int lPos = 15;

  String xSub = input.substring(xPos + 1, yPos);
  x_value = (signed char)xSub.toInt();
  String ySub = input.substring(yPos + 1, sPos);
  y_value = (signed char)ySub.toInt();
  String sSub = input.substring(sPos + 1, lPos);
  s_value = (signed char)sSub.toInt();
  String lSub = input.substring(lPos + 1);
  l_value = (signed char)lSub.toInt();
}

//functions are renamed for now to make sure the compiler doesn't cry
void setup_temp() {
  Serial.begin(115200);
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);

  server.on("/status", HTTP_GET, [](AsyncWebServerRequest *request) {
    JsonDocument data;
    data["name"] = ssid;
    data["battery"] = battery;
    battery++;
    String response;
    serializeJson(data, response);
    request->send(200, "application/json", response);
  });

  server.begin();

  if(udp.listen(3001)) {
    udp.onPacket([](AsyncUDPPacket packet) {
      parseControllerInput(String(packet.data(), packet.length()));
      //Serial.write(packet.data(), packet.length());
    });
  }
}

// not needed
void loop_temp() {
  Serial.println(x_value);
  delay(100);
}