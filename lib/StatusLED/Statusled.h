/*
  StatusLED.h
  Simple non-blocking RGB status indicator for the "Cheap Yellow Display"
  (ESP32-2432S028R) onboard LED.

  Hardware assumptions (CYD ESP32-2432S028R):
    - Red   = GPIO 4
    - Green = GPIO 16
    - Blue  = GPIO 17
    - Active-LOW: writing LOW turns a channel ON, HIGH turns it OFF.

  If your board wires the LED differently, change PIN_R/PIN_G/PIN_B
  and ACTIVE_LOW in StatusLED.cpp — the API doesn't change.

  Usage:
    #include <StatusLED.h>

    void setup() {
      StatusLED.begin();
      StatusLED.ok();
    }

    void loop() {
      StatusLED.update();   // MUST be called every loop iteration
      ...
      if (somethingBad) StatusLED.error();
    }
*/

#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <Arduino.h>

enum StatusColor {
  LED_OFF     = 0,
  LED_RED     = 1,
  LED_GREEN   = 2,
  LED_BLUE    = 3,
  LED_YELLOW  = 4,  // R+G
  LED_CYAN    = 5,  // G+B
  LED_MAGENTA = 6,  // R+B
  LED_WHITE   = 7   // R+G+B
};

class StatusLEDClass {
public:
  void begin();

  // Call every loop() iteration. Handles blinking; returns immediately
  // if the current state is solid or off (no delay, no blocking).
  void update();

  // ---- Presets ----
  void ok();       // solid green
  void warning();  // amber, slow blink (~1 cycle/sec)
  void error();     // red, fast blink (~2 cycles/sec)
  void busy();      // blue, rapid flicker
  void boot();      // solid blue (for booting)
  void off();

  // ---- Escape hatches ----
  void setSolid(StatusColor c);

  // intervalMs = duration of each on/off phase (full cycle = 2x intervalMs)
  void setBlink(StatusColor c, unsigned long intervalMs);

  // Convenience: specify in full blink cycles per second, e.g. custom(LED_RED, 2.0)
  void custom(StatusColor c, float blinksPerSecond);

private:
  static const uint8_t PIN_R = 4;
  static const uint8_t PIN_G = 16;
  static const uint8_t PIN_B = 17;
  static const bool ACTIVE_LOW = true;

  StatusColor _color = LED_OFF;
  unsigned long _blinkIntervalMs = 0;
  unsigned long _lastToggle = 0;
  bool _blinkOn = true;

  void _writeChannel(uint8_t pin, bool on);
  void _applyRaw(StatusColor c);
};

extern StatusLEDClass StatusLED;

#endif // STATUS_LED_H