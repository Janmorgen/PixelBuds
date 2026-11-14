#pragma once

#include <Arduino.h>
#include <ArduinoOTA.h>
#include <FastLED.h>
#include <WebServer.h>
#include <WiFi.h>

#include "daycycle.h"
#include "secrets.h"

// Wi-Fi station connection, network time, wireless flashing (ArduinoOTA) and a plain-text status server.

#define PIXELBUDS_HOSTNAME "pixelbuds"
#define WIFI_RETRY_INTERVAL_MS 30000

static uint32_t wifiLastAttempt = 0;
static bool wifiWasConnected = false;
static bool otaStarted = false;
static bool webStarted = false;
static WebServer web(80);

// Serves status at /, starts a test day at /test and ends it at /test/stop.
static void webBegin() {
  web.on("/", []() { web.send(200, "text/plain", dayCycleStatus()); });
  web.on("/test", []() {
    startTestDay();
    dayCycleUpdate();
    web.send(200, "text/plain", "Test day started: 05:00 to 05:00 in 24 minutes, one minute per hour.\n\n" + dayCycleStatus());
  });
  web.on("/test/stop", []() {
    stopTestDay();
    dayCycleUpdate();
    web.send(200, "text/plain", "Test day stopped.\n\n" + dayCycleStatus());
  });
  web.begin();
  webStarted = true;
}

// Registers the OTA callbacks and starts listening; needs a live Wi-Fi link.
static void otaBegin() {
  ArduinoOTA.setHostname(PIXELBUDS_HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]() {
    FastLED.clear(true);
    Serial.println("OTA started");
  });
  ArduinoOTA.onProgress([](unsigned int done, unsigned int total) {
    static unsigned int lastPercent = 101;
    unsigned int percent = total ? (done * 100UL) / total : 0;
    if (percent / 10 != lastPercent / 10) Serial.printf("OTA %u%%\n", percent);
    lastPercent = percent;
  });
  ArduinoOTA.onEnd([]() { Serial.println("OTA done, rebooting"); });
  ArduinoOTA.onError([](ota_error_t error) { Serial.printf("OTA failed: error %u\n", error); });

  ArduinoOTA.begin();
  otaStarted = true;
}

// Starts connecting to Wi-Fi without waiting for the link.
void networkBegin() {
  WiFi.setHostname(PIXELBUDS_HOSTNAME);
  WiFi.mode(WIFI_STA);
  WiFi.setSleep(false);
  WiFi.setAutoReconnect(true);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  wifiLastAttempt = millis();
  configTzTime(TZ_INFO, "pool.ntp.org", "time.google.com");
}

// Keeps Wi-Fi up, starts OTA once connected and services OTA requests; call every loop.
void networkLoop() {
  if (WiFi.status() == WL_CONNECTED) {
    if (!wifiWasConnected) {
      wifiWasConnected = true;
      Serial.printf("Wi-Fi up at %s (%s.local)\n", WiFi.localIP().toString().c_str(), PIXELBUDS_HOSTNAME);
      if (!otaStarted) otaBegin();
      if (!webStarted) webBegin();
    }
    ArduinoOTA.handle();
    web.handleClient();
    return;
  }

  if (wifiWasConnected) {
    wifiWasConnected = false;
    Serial.println("Wi-Fi lost, retrying");
  }
  if (millis() - wifiLastAttempt >= WIFI_RETRY_INTERVAL_MS) {
    wifiLastAttempt = millis();
    WiFi.disconnect();
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  }
}
