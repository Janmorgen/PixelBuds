// Frame status output: frameJson() is valid JSON with every field, and frameStatus() is one whole line, at boot and
// at the widest values the counters can reach.
#include "common.h"
#include "frameclock.h"
#include <cctype>

// A small JSON checker: true when `s` from `p` holds one value, read up to `p`.
static bool jsonValue(const char*& p);
static void skipSpace(const char*& p) { while (*p == ' ' || *p == '\n' || *p == '\t' || *p == '\r') p++; }
static bool jsonString(const char*& p) {
  if (*p != '"') return false;
  for (p++; *p && *p != '"'; p++) {
    if ((unsigned char)*p < 0x20) return false;
    if (*p == '\\' && !*++p) return false;
  }
  if (*p != '"') return false;
  p++;
  return true;
}
static bool jsonNumber(const char*& p) {
  const char* start = p;
  if (*p == '-') p++;
  if (!isdigit((unsigned char)*p)) return false;
  if (*p == '0' && isdigit((unsigned char)p[1])) return false;
  while (isdigit((unsigned char)*p)) p++;
  if (*p == '.') { p++; if (!isdigit((unsigned char)*p)) return false; while (isdigit((unsigned char)*p)) p++; }
  if (*p == 'e' || *p == 'E') {
    p++;
    if (*p == '+' || *p == '-') p++;
    if (!isdigit((unsigned char)*p)) return false;
    while (isdigit((unsigned char)*p)) p++;
  }
  return p > start;
}
static bool jsonObject(const char*& p) {
  if (*p != '{') return false;
  p++; skipSpace(p);
  if (*p == '}') { p++; return true; }
  for (;;) {
    skipSpace(p);
    if (!jsonString(p)) return false;
    skipSpace(p);
    if (*p++ != ':') return false;
    if (!jsonValue(p)) return false;
    skipSpace(p);
    if (*p == '}') { p++; return true; }
    if (*p++ != ',') return false;
  }
}
static bool jsonValue(const char*& p) {
  skipSpace(p);
  if (*p == '{') return jsonObject(p);
  if (*p == '"') return jsonString(p);
  if (!strncmp(p, "true", 4)) { p += 4; return true; }
  if (!strncmp(p, "false", 5)) { p += 5; return true; }
  if (!strncmp(p, "null", 4)) { p += 4; return true; }
  return jsonNumber(p);
}
static bool validJson(const std::string& s) {
  const char* p = s.c_str();
  if (!jsonValue(p)) return false;
  skipSpace(p);
  return *p == 0;
}

static void checkOutput(const char* label) {
  std::string json = "{" + frameJson() + "}";
  std::string line = frameStatus();
  char late[16];
  snprintf(late, sizeof(late), "%lu", (unsigned long)frameStats.late);
  printf("%s:\n  %s\n  %s", label, json.c_str(), line.c_str());
  CHECK(validJson(json), "%s: frameJson() is not valid JSON: %s", label, json.c_str());
  for (const char* key : {"\"frames\":{", "\"fps\":", "\"targetFps\":", "\"workAvgMs\":", "\"workMaxMs\":", "\"late\":"})
    CHECK(json.find(key) != std::string::npos, "%s: frameJson() lacks %s", label, key);
  CHECK(json.find(std::string("\"late\":") + late + "}}") != std::string::npos, "%s: frameJson() does not end with late %s",
        label, late);
  CHECK(!line.empty() && line.back() == '\n' && line.find('\n') == line.size() - 1,
        "%s: frameStatus() is not one line ending in a newline (%zu chars)", label, line.size());
  CHECK(line.find(std::string("late ") + late + "\n") != std::string::npos,
        "%s: frameStatus() does not end with late %s and a newline", label, late);
}

int main() {
  // The checker itself.
  CHECK(validJson("{\"a\":{\"b\":-1.5e3,\"c\":\"x\"},\"d\":true}"), "the JSON checker rejects valid JSON");
  for (const char* bad : {"{\"a\":1,}", "{\"a\":nan}", "{\"a\":inf}", "{\"a\":1", "{\"a\":01}", "{\"a\":1}}", "{\"a\" 1}"})
    CHECK(!validJson(bad), "the JSON checker accepts %s", bad);

  frameStats = {0, 0, 0, 0};
  checkOutput("at boot");
  // Widest values: fps above its 1e6 / SHOW_GAP_US ceiling, the most work a uint32_t of microseconds holds, the
  // largest late count.
  frameStats = {1000.0f, UINT32_MAX / 1000.0f, UINT32_MAX / 1000.0f, UINT32_MAX};
  checkOutput("widest values");
  frameStats = {53.3f, 1.23f, 2.34f, 7};
  checkOutput("typical");
  printf(failures ? "status: %d failure(s)\n" : "status: ok\n", failures);
  return failures != 0;
}
