#include "StatusLED.h"

void StatusLEDClass::begin() {
  pinMode(PIN_R, OUTPUT);
  pinMode(PIN_G, OUTPUT);
  pinMode(PIN_B, OUTPUT);
  off();
}

void StatusLEDClass::update() {
  if (_blinkIntervalMs == 0) return; // solid or off, nothing to animate

  unsigned long now = millis();
  if (now - _lastToggle >= _blinkIntervalMs) {
    _lastToggle = now;
    _blinkOn = !_blinkOn;
    _applyRaw(_blinkOn ? _color : LED_OFF);
  }
}

void StatusLEDClass::ok()      { setSolid(LED_GREEN); }
void StatusLEDClass::warning() { setBlink(LED_YELLOW, 500); }  // 1 cycle/sec
void StatusLEDClass::error()   { setBlink(LED_RED, 250); }     // 2 cycles/sec
void StatusLEDClass::busy()    { setBlink(LED_BLUE, 150); }    // fast flicker
void StatusLEDClass::off()     { setSolid(LED_OFF); }
void StatusLEDClass::boot()    { setSolid(LED_BLUE); }

void StatusLEDClass::setSolid(StatusColor c) {
  _color = c;
  _blinkIntervalMs = 0;
  _applyRaw(c);
}

void StatusLEDClass::setBlink(StatusColor c, unsigned long intervalMs) {
  _color = c;
  _blinkIntervalMs = intervalMs;
  _blinkOn = true;
  _lastToggle = millis();
  _applyRaw(c);
}

void StatusLEDClass::custom(StatusColor c, float blinksPerSecond) {
  unsigned long intervalMs = (unsigned long)(500.0f / blinksPerSecond);
  setBlink(c, intervalMs);
}

void StatusLEDClass::_writeChannel(uint8_t pin, bool on) {
  bool level = ACTIVE_LOW ? !on : on;
  digitalWrite(pin, level ? HIGH : LOW);
}

void StatusLEDClass::_applyRaw(StatusColor c) {
  bool r = (c == LED_RED || c == LED_YELLOW || c == LED_MAGENTA || c == LED_WHITE);
  bool g = (c == LED_GREEN || c == LED_YELLOW || c == LED_CYAN || c == LED_WHITE);
  bool b = (c == LED_BLUE || c == LED_CYAN || c == LED_MAGENTA || c == LED_WHITE);
  _writeChannel(PIN_R, r);
  _writeChannel(PIN_G, g);
  _writeChannel(PIN_B, b);
}

StatusLEDClass StatusLED;