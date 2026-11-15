#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <initializer_list>

#include "daycycle.h"
#include "islands.h"

// Plain-text HTTP API on port 80. Every handler checks all its arguments before changing anything.

static WebServer web(80);
static bool webStarted = false;

// ---------- argument checking ----------

// Collects the first problem found while checking a request.
struct Check {
  String error;
  bool ok() const { return error.length() == 0; }
  void fail(const String& msg) { if (ok()) error = msg; }
};

// Fails on any argument not in `allowed`.
static void allowOnly(Check& c, std::initializer_list<const char*> allowed) {
  for (int i = 0; i < web.args(); i++) {
    String name = web.argName(i);
    if (name == "plain") continue;   // POST body, added by WebServer
    bool known = false;
    for (const char* a : allowed) if (name == a) known = true;
    if (!known) {
      String list;
      for (const char* a : allowed) list += String(list.length() ? ", " : "") + a;
      c.fail("Unknown option '" + name + "'. Options here: " + (list.length() ? list : "none") + ".");
    }
  }
}

// Whole-string integer within [lo, hi].
static bool parseInt(const String& s, long lo, long hi, long& out) {
  if (s.length() == 0 || s.length() > 9) return false;
  int i = (s[0] == '-' || s[0] == '+') ? 1 : 0;
  if (i == (int)s.length()) return false;
  for (int k = i; k < (int)s.length(); k++) if (!isDigit(s[k])) return false;
  out = s.toInt();
  return out >= lo && out <= hi;
}

// Whole-string decimal number within [lo, hi].
static bool parseNumber(const String& s, float lo, float hi, float& out) {
  if (s.length() == 0 || s.length() > 9) return false;
  bool dot = false, digit = false;
  for (int k = 0; k < (int)s.length(); k++) {
    if (isDigit(s[k])) digit = true;
    else if (s[k] == '.' && !dot) dot = true;
    else return false;
  }
  if (!digit) return false;
  out = s.toFloat();
  return out >= lo && out <= hi;
}

// Strict 24-hour H:MM or HH:MM, as minutes after midnight.
static bool parseClock(const String& s, int& minute) {
  int colon = s.indexOf(':');
  if (colon < 1 || colon > 2 || (int)s.length() != colon + 3) return false;
  long h, m;
  if (!parseInt(s.substring(0, colon), 0, 23, h) || s[0] == '-' || s[0] == '+') return false;
  if (!isDigit(s[colon + 1]) || !isDigit(s[colon + 2])) return false;
  m = s.substring(colon + 1).toInt();
  if (m > 59) return false;
  minute = h * 60 + m;
  return true;
}

// Index of `s` in `names`, or -1.
static int pickName(const String& s, const char* const* names, int count) {
  for (int i = 0; i < count; i++) if (s == names[i]) return i;
  return -1;
}

static void sendText(int code, const String& body) {
  web.send(code, "text/plain", body.endsWith("\n") ? body : body + "\n");
}

// Sends 400 with the check's error; returns true if it did.
static bool rejected(const Check& c) {
  if (c.ok()) return false;
  sendText(400, c.error);
  return true;
}

// ---------- status ----------

static String uptimeText() {
  uint32_t s = millis() / 1000;
  char buf[32];
  snprintf(buf, sizeof(buf), "%lud %02lu:%02lu:%02lu", (unsigned long)(s / 86400), (unsigned long)(s / 3600 % 24),
           (unsigned long)(s / 60 % 60), (unsigned long)(s % 60));
  return buf;
}

static String statusText() {
  return dayCycleStatus() + islandStatus(activeLook.islandHold) +
         "uptime:      " + uptimeText() + "  build " __DATE__ " " __TIME__ "\n";
}

static String statusJson() {
  return "{" + dayCycleJson() + "," + islandJson(activeLook.islandHold) +
         ",\"uptimeSeconds\":" + String(millis() / 1000) + ",\"build\":\"" __DATE__ " " __TIME__ "\"}";
}

static const char HELP_TEXT[] =
  "PixelBuds HTTP API. Plain text unless noted; every route answers GET.\n"
  "\n"
  "/ or /status          status\n"
  "/status.json          status as JSON\n"
  "/help                 this list\n"
  "/test                 24-minute test day from 05:00\n"
  "/test?hold=day|HH:MM  hold the light at midday or a time, until /test/stop\n"
  "/test/stop            end a test day or hold\n"
  "/islands              island experiment: ?mode=off|split|edge|orphan  ?round=on|off\n";

// ---------- routes ----------

static void handleStatus() {
  Check c;
  allowOnly(c, {});
  if (rejected(c)) return;
  sendText(200, statusText());
}

static void handleStatusJson() {
  web.send(200, "application/json", statusJson());
}

static void handleIslands() {
  Check c;
  allowOnly(c, {"mode", "round"});
  int mode = -1, round = -1;
  if (web.hasArg("mode") && (mode = pickName(web.arg("mode"), islandModeNames, 4)) < 0)
    c.fail("Unknown mode. Use off, split, edge or orphan.");
  if (web.hasArg("round") && (round = pickName(web.arg("round"), onOffNames, 2)) < 0)
    c.fail("Unknown rounding. Use on or off.");
  if (rejected(c)) return;

  if (mode >= 0) requestIslandMode(mode);
  if (round >= 0) islandRounding = round == 1;
  sendText(200, islandStatus(activeLook.islandHold));
}

static void handleTest() {
  Check c;
  allowOnly(c, {"hold"});
  int minute = -1;
  if (web.hasArg("hold")) {
    String h = web.arg("hold");
    if (h == "day") minute = 720;
    else if (!parseClock(h, minute)) c.fail("Use /test?hold=day or /test?hold=HH:MM (00:00 to 23:59).");
  }
  if (rejected(c)) return;

  if (minute >= 0) {
    startTestHold(minute);
    dayCycleUpdate();
    sendText(200, "Holding the light at " + web.arg("hold") + " until /test/stop.\n\n" + statusText());
    return;
  }
  startTestDay();
  dayCycleUpdate();
  sendText(200, "Test day started: 05:00 to 05:00 in 24 minutes, one minute per hour.\n\n" + statusText());
}

static void handleTestStop() {
  Check c;
  allowOnly(c, {});
  if (rejected(c)) return;
  stopTestDay();
  dayCycleUpdate();
  sendText(200, "Test day stopped.\n\n" + statusText());
}

// Registers the routes and starts the server; needs a live Wi-Fi link.
static void webBegin() {
  web.on("/", handleStatus);
  web.on("/status", handleStatus);
  web.on("/status.json", handleStatusJson);
  web.on("/help", []() { sendText(200, HELP_TEXT); });
  web.on("/islands", handleIslands);
  web.on("/test", handleTest);
  web.on("/test/stop", handleTestStop);
  web.onNotFound([]() { sendText(404, "No such endpoint: " + web.uri() + "\n\n" + HELP_TEXT); });
  web.begin();
  webStarted = true;
}
