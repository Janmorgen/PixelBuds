#pragma once

#include <Arduino.h>
#include <WebServer.h>
#include <initializer_list>

#include "daycycle.h"
#include "islands.h"
#include "overrides.h"
#include "rules.h"

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
  return dayCycleStatus() + overridesStatus() + rulesStatus() + islandStatus(currentIslandHold()) +
         "uptime:      " + uptimeText() + "  build " __DATE__ " " __TIME__ "\n";
}

static String statusJson() {
  return "{" + dayCycleJson() + "," + overridesJson() + "," + rulesJson() + "," + islandJson(currentIslandHold()) +
         ",\"uptimeSeconds\":" + String(millis() / 1000) + ",\"build\":\"" __DATE__ " " __TIME__ "\"}";
}

static const char HELP_TEXT[] =
  "PixelBuds HTTP API. Plain text unless noted; every route answers GET.\n"
  "\n"
  "/ or /status          status\n"
  "/status.json          status as JSON\n"
  "/help                 this list\n"
  "\n"
  "Overrides. They last until the phase changes (status shows when) or /reset:\n"
  "/power/off            ?fade=SEC (0-600, default 1)  ?until=morning keeps it off until wake\n"
  "/power/on             ?fade=SEC; also cancels a sleep timer\n"
  "/brightness?scale=N   0-100 percent of the current look\n"
  "/sleep?in=MIN         dim to off over 1-240 minutes, then off until wake\n"
  "/adjust               ?speed= ?pixels= ?trail= ?mix=  percent of the look (25-400; pixels 25-200)\n"
  "/colors               ?red= ?green= ?blue= ?amber=  0-100, or auto for the schedule's\n"
  "/phase/next           jump to the next look now; the schedule catches up\n"
  "/pause/on, /pause/off freeze or resume the motion\n"
  "/shuffle              rescatter the pixels\n"
  "/reset                clear every override, power-off and skip included\n"
  "\n"
  "Testing:\n"
  "/test                 24-minute test day from 05:00\n"
  "/test?hold=X          X = day, golden, evening, embers or HH:MM; ?for=MIN (1-1440) ends it\n"
  "/test/stop            end a test day or hold\n"
  "\n"
  "Settings, kept until changed:\n"
  "/rules                ?mix=pool|pair  ?mixrate=0-100  ?life=MIN-MAX (contact frames, 1-2000)\n"
  "                      ?expire=snap|fade  ?round=on|off\n"
  "/rules/reset          back to pool, 100, 30-100, snap, off\n"
  "/islands              ?mode=off|split|edge|orphan  ?boost=0-5  ?hold=auto|0-100  (?round= still works)\n"
  "/islands/reset        back to split, boost 2, hold auto\n";

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
  allowOnly(c, {"mode", "round", "boost", "hold"});
  int mode = -1, round = -1;
  float boost = -1, hold = -2;
  if (web.hasArg("mode") && (mode = pickName(web.arg("mode"), islandModeNames, 4)) < 0)
    c.fail("Unknown mode. Use off, split, edge or orphan.");
  if (web.hasArg("round") && (round = pickName(web.arg("round"), onOffNames, 2)) < 0)
    c.fail("Unknown rounding. Use on or off.");
  if (web.hasArg("boost") && !parseNumber(web.arg("boost"), 0, 5, boost)) c.fail("?boost= takes 0 to 5.");
  if (web.hasArg("hold")) {
    long pct;
    if (web.arg("hold") == "auto") hold = -1;
    else if (parseInt(web.arg("hold"), 0, 100, pct)) hold = pct / 100.0f;
    else c.fail("?hold= takes auto or 0 to 100.");
  }
  if (rejected(c)) return;

  if (mode >= 0) requestIslandMode(mode);
  if (round >= 0) ruleRound = round == 1;
  if (boost >= 0) islandBoost = boost;
  if (hold > -2) islandHoldSetting = hold;
  sendText(200, islandStatus(currentIslandHold()) + rulesStatus());
}

static void handleRules() {
  Check c;
  allowOnly(c, {"mix", "mixrate", "life", "expire", "round"});
  int mix = -1, expire = -1, round = -1;
  long rate = -1, lifeMin = -1, lifeMax = -1;
  if (web.hasArg("mix") && (mix = pickName(web.arg("mix"), mixNames, 2)) < 0) c.fail("?mix= takes pool or pair.");
  if (web.hasArg("mixrate") && !parseInt(web.arg("mixrate"), 0, 100, rate)) c.fail("?mixrate= takes 0 to 100.");
  if (web.hasArg("expire") && (expire = pickName(web.arg("expire"), expireNames, 2)) < 0)
    c.fail("?expire= takes snap or fade.");
  if (web.hasArg("round") && (round = pickName(web.arg("round"), onOffNames, 2)) < 0) c.fail("?round= takes on or off.");
  if (web.hasArg("life")) {
    String l = web.arg("life");
    int dash = l.indexOf('-');
    if (dash < 1 || !parseInt(l.substring(0, dash), 1, RULE_LIFE_LIMIT, lifeMin) ||
        !parseInt(l.substring(dash + 1), 1, RULE_LIFE_LIMIT, lifeMax) || lifeMin > lifeMax)
      c.fail("?life= takes MIN-MAX, 1 to 2000 with MIN <= MAX, e.g. 30-100.");
  }
  if (rejected(c)) return;

  if (mix >= 0) ruleMix = mix;
  if (rate >= 0) ruleMixRate = rate;
  if (expire >= 0) ruleExpireFade = expire == 1;
  if (round >= 0) ruleRound = round == 1;
  if (lifeMin >= 0) { ruleLifeMin = lifeMin; ruleLifeMax = lifeMax; }
  sendText(200, rulesStatus());
}

static void handleTest() {
  Check c;
  allowOnly(c, {"hold", "for"});
  int minute = -1, look = -1;
  long forMin = 0;
  if (web.hasArg("hold")) {
    String h = web.arg("hold");
    look = pickName(h, lookNames, LOOK_COUNT);
    if (look < 0 && !parseClock(h, minute))
      c.fail("Use /test?hold= with day, golden, evening, embers or HH:MM (00:00 to 23:59).");
  }
  if (web.hasArg("for")) {
    if (!web.hasArg("hold")) c.fail("?for= goes with ?hold=.");
    else if (!parseInt(web.arg("for"), 1, 1440, forMin)) c.fail("?for= takes minutes, 1 to 1440.");
  }
  if (rejected(c)) return;

  if (web.hasArg("hold")) {
    startTestHold(minute, look, forMin * 60000UL);
    dayCycleUpdate();
    String until = forMin ? "for " + String(forMin) + " min" : String("until /test/stop");
    sendText(200, "Holding " + web.arg("hold") + " " + until + ".\n\n" + statusText());
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

// Reads ?fade=SEC (default 1 s) into ms.
static void readFade(Check& c, uint32_t& ms) {
  long sec = 1;
  if (web.hasArg("fade") && !parseInt(web.arg("fade"), 0, 600, sec)) c.fail("?fade= takes seconds, 0 to 600.");
  ms = sec * 1000;
}

static void handlePowerOff() {
  Check c;
  allowOnly(c, {"fade", "until"});
  uint32_t fadeMs;
  readFade(c, fadeMs);
  bool morning = false;
  if (web.hasArg("until")) {
    if (web.arg("until") == "morning") morning = true;
    else if (web.arg("until") != "phase") c.fail("?until= takes morning or phase.");
  }
  if (rejected(c)) return;
  setPower(false, morning, fadeMs);
  sendText(200, "Off.\n\n" + overridesStatus());
}

static void handlePowerOn() {
  Check c;
  allowOnly(c, {"fade"});
  uint32_t fadeMs;
  readFade(c, fadeMs);
  if (rejected(c)) return;
  setPower(true, false, fadeMs);
  sendText(200, "On.\n\n" + overridesStatus());
}

static void handleBrightness() {
  Check c;
  allowOnly(c, {"scale"});
  long pct = -1;
  if (web.hasArg("scale") && !parseInt(web.arg("scale"), 0, 100, pct)) c.fail("?scale= takes a percentage, 0 to 100.");
  if (rejected(c)) return;
  if (pct >= 0) setBrightnessPct(pct);
  sendText(200, overridesStatus());
}

static void handleSleep() {
  Check c;
  allowOnly(c, {"in"});
  long minutes = 0;
  if (!web.hasArg("in")) c.fail("Use /sleep?in=MIN, 1 to 240.");
  else if (!parseInt(web.arg("in"), 1, 240, minutes)) c.fail("?in= takes minutes, 1 to 240.");
  if (rejected(c)) return;
  startSleep(minutes);
  sendText(200, "Sleeping.\n\n" + overridesStatus());
}

static void handleAdjust() {
  Check c;
  const char* names[] = {"speed", "pixels", "trail", "mix"};
  int* targets[] = {&adjustSpeed, &adjustPixels, &adjustTrail, &adjustMix};
  long values[4];
  allowOnly(c, {"speed", "pixels", "trail", "mix"});
  for (int i = 0; i < 4; i++) {
    values[i] = -1;
    long hi = i == 1 ? 200 : 400;
    if (web.hasArg(names[i]) && !parseInt(web.arg(names[i]), 25, hi, values[i]))
      c.fail(String("?") + names[i] + "= takes a percentage, 25 to " + hi + ".");
  }
  if (rejected(c)) return;
  for (int i = 0; i < 4; i++) if (values[i] >= 0) *targets[i] = values[i];
  sendText(200, overridesStatus());
}

static void handleColors() {
  Check c;
  allowOnly(c, {"red", "green", "blue", "amber"});
  float values[PALETTE_SIZE];
  bool anyPresent = false;
  for (int i = 0; i < PALETTE_SIZE; i++) {
    values[i] = colourOverride[i];
    bool scheduled = activeSegFrom->weights[i] > 0 || activeSegTo->weights[i] > 0;
    if (web.hasArg(paletteNames[i])) {
      long pct;
      if (web.arg(paletteNames[i]) == "auto") values[i] = -1;
      else if (parseInt(web.arg(paletteNames[i]), 0, 100, pct)) values[i] = pct / 100.0f;
      else c.fail(String("?") + paletteNames[i] + "= takes 0 to 100, or auto.");
    }
    anyPresent |= values[i] > 0 || (values[i] < 0 && scheduled);
  }
  if (c.ok() && !anyPresent) c.fail("At least one colour has to stay on.");
  if (rejected(c)) return;
  for (int i = 0; i < PALETTE_SIZE; i++) colourOverride[i] = values[i];
  sendText(200, overridesStatus());
}

static void handlePhaseNext() {
  Check c;
  allowOnly(c, {});
  if (rejected(c)) return;
  const char* look;
  float at;
  if (!skipToNextLook(&look, &at)) {
    sendText(409, "Can't skip ahead during a test day or hold, or before the clock is set.");
    return;
  }
  char t[8];
  formatMinute(t, sizeof(t), at);
  dayCycleUpdate();
  sendText(200, String("Showing the ") + look + " look now; the schedule catches up at " + t + ".\n\n" + statusText());
}

// Answers a route that takes no options; returns false if it had some.
static bool noOptions() {
  Check c;
  allowOnly(c, {});
  return !rejected(c);
}

// Registers the routes and starts the server; needs a live Wi-Fi link.
static void webBegin() {
  web.on("/", handleStatus);
  web.on("/status", handleStatus);
  web.on("/status.json", handleStatusJson);
  web.on("/help", []() { sendText(200, HELP_TEXT); });
  web.on("/islands", handleIslands);
  web.on("/islands/reset", []() { if (noOptions()) { resetIslands(); sendText(200, islandStatus(currentIslandHold())); } });
  web.on("/rules", handleRules);
  web.on("/rules/reset", []() { if (noOptions()) { resetRules(); sendText(200, rulesStatus()); } });
  web.on("/test", handleTest);
  web.on("/test/stop", handleTestStop);
  web.on("/power/off", handlePowerOff);
  web.on("/power/on", handlePowerOn);
  web.on("/brightness", handleBrightness);
  web.on("/sleep", handleSleep);
  web.on("/adjust", handleAdjust);
  web.on("/colors", handleColors);
  web.on("/phase/next", handlePhaseNext);
  web.on("/pause/on", []() { if (noOptions()) { paused = true; sendText(200, "Paused.\n\n" + overridesStatus()); } });
  web.on("/pause/off", []() { if (noOptions()) { paused = false; sendText(200, "Resumed.\n\n" + overridesStatus()); } });
  web.on("/shuffle", []() { if (noOptions()) { shuffleRequested = true; sendText(200, "Shuffled."); } });
  web.on("/reset", []() { if (noOptions()) { resetOverrides(); dayCycleUpdate(); sendText(200, "Overrides cleared.\n\n" + statusText()); } });
  web.onNotFound([]() { sendText(404, "No such endpoint: " + web.uri() + "\n\n" + HELP_TEXT); });
  web.begin();
  webStarted = true;
}
