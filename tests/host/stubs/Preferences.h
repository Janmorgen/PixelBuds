#pragma once
struct Preferences {
  bool begin(const char*, bool) { return true; } void end() {}
  bool isKey(const char*) { return false; }
  int getInt(const char*, int d = 0) { return d; } float getFloat(const char*, float d = 0) { return d; }
  bool getBool(const char*, bool d = false) { return d; }
  void putInt(const char*, int) {} void putFloat(const char*, float) {} void putBool(const char*, bool) {} bool remove(const char*) { return true; }
};
