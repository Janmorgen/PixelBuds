#pragma once
#include <cstdint>
#include <cmath>
#include <cstring>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <string>
#include <vector>
#include <algorithm>
#include <functional>
#include <random>
using std::min; using std::max;

extern uint64_t simMicros;   // virtual clock; the loop's own work takes no time
inline uint32_t millis() { return (uint32_t)(simMicros / 1000); }
inline uint32_t micros() { return (uint32_t)simMicros; }
inline void delay(uint32_t ms) { simMicros += (uint64_t)ms * 1000; }
inline void delayMicroseconds(uint32_t us) { simMicros += us; }
inline time_t simTime(time_t*) { return 0; }                // clock never set; the sim pins a look
#define time(x) simTime(x)

extern std::mt19937 simRng;
inline long random(long n) { return n <= 0 ? 0 : (long)(simRng() % (uint32_t)n); }
inline long random(long a, long b) { return b <= a ? a : a + random(b - a); }
inline void randomSeed(unsigned long) {}   // the sim seeds simRng from its seed argument
inline int analogRead(int) { return 0; }
#define OUTPUT 1
#define LOW 0
inline void pinMode(int, int) {}
inline void digitalWrite(int, int) {}
inline bool isDigit(char c) { return c >= '0' && c <= '9'; }
#define constrain(x, lo, hi) ((x) < (lo) ? (lo) : ((x) > (hi) ? (hi) : (x)))
struct SerialStub { void begin(int) {} void println(const char*) {} template <class... A> void printf(const char*, A...) {} };
static SerialStub Serial;

class String : public std::string {
 public:
  String() {}
  String(const char* s) : std::string(s ? s : "") {}
  String(const std::string& s) : std::string(s) {}
  String(char c) : std::string(1, c) {}
  String(int v) : std::string(std::to_string(v)) {}
  String(unsigned v) : std::string(std::to_string(v)) {}
  String(long v) : std::string(std::to_string(v)) {}
  String(unsigned long v) : std::string(std::to_string(v)) {}
  String(float v, int digits = 2) { char b[32]; snprintf(b, sizeof b, "%.*f", digits, v); assign(b); }
  String(double v, int digits = 2) { char b[32]; snprintf(b, sizeof b, "%.*f", digits, v); assign(b); }
  int length() const { return (int)size(); }
  int indexOf(char c, int from = 0) const { auto p = find(c, from); return p == npos ? -1 : (int)p; }
  String substring(int a, int b = -1) const { if (b < 0) b = size(); return String(std::string::substr(a, b - a)); }
  long toInt() const { return atol(c_str()); }
  float toFloat() const { return atof(c_str()); }
  bool endsWith(const String& s) const { return size() >= s.size() && compare(size() - s.size(), s.size(), s) == 0; }
  String& operator+=(const String& o) { append(o); return *this; }
  String& operator+=(const char* o) { append(o); return *this; }
  String& operator+=(char c) { push_back(c); return *this; }
};
inline String operator+(const String& a, const String& b) { String r(a); r.append(b); return r; }
inline String operator+(const String& a, const char* b) { String r(a); r.append(b); return r; }
inline String operator+(const char* a, const String& b) { String r(a); r.append(b); return r; }
inline String operator+(const String& a, int b) { return a + String(b); }
inline String operator+(const String& a, long b) { return a + String(b); }
inline String operator+(const String& a, unsigned long b) { return a + String(b); }
inline String operator+(const String& a, unsigned b) { return a + String(b); }
inline String operator+(const String& a, char b) { return a + String(b); }
