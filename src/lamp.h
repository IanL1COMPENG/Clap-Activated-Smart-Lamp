// RGB lamp: on/off, colour palette, gamma-corrected cross-fades, and state
// persisted in flash (NVS) so the lamp comes back the way it was left.
#pragma once

#include <Arduino.h>
#include <Preferences.h>

class Lamp {
 public:
  void begin();
  void toggle();
  void nextColor();       // also turns the lamp on
  void update(uint32_t nowMs);  // call often: runs the fade and deferred save

  bool isOn() const { return on_; }
  uint8_t colorIndex() const { return color_; }
  const char* colorName() const;

 private:
  struct Rgb { float r, g, b; };
  void startFade(uint32_t nowMs);
  void write(const Rgb& c);
  Rgb target() const;

  Preferences prefs_;
  bool on_ = false;
  uint8_t color_ = 0;
  Rgb from_{0, 0, 0}, current_{0, 0, 0};
  uint32_t fadeStart_ = 0;
  bool fading_ = false;
  bool dirty_ = false;
  uint32_t changedAt_ = 0;
};
