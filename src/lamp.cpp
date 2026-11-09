#include "lamp.h"

#include <math.h>

#include "config.h"

namespace {

struct Swatch {
  const char* name;
  uint8_t r, g, b;
};

// Colours chosen to look distinct on a typical RGB LED (values before gamma).
constexpr Swatch kPalette[] = {
    {"warm white", 255, 150, 60}, {"red", 255, 0, 0},     {"orange", 255, 70, 0},
    {"yellow", 255, 180, 0},      {"green", 0, 255, 30},  {"cyan", 0, 220, 200},
    {"blue", 0, 40, 255},         {"purple", 130, 0, 255}, {"pink", 255, 30, 110},
};
constexpr uint8_t kColors = sizeof(kPalette) / sizeof(kPalette[0]);

constexpr uint8_t CH_R = 0, CH_G = 1, CH_B = 2;

float gamma22(float v) { return powf(v, 2.2f); }

}  // namespace

void Lamp::begin() {
  ledcSetup(CH_R, cfg::PWM_FREQ_HZ, cfg::PWM_BITS);
  ledcSetup(CH_G, cfg::PWM_FREQ_HZ, cfg::PWM_BITS);
  ledcSetup(CH_B, cfg::PWM_FREQ_HZ, cfg::PWM_BITS);
  ledcAttachPin(cfg::PIN_RED, CH_R);
  ledcAttachPin(cfg::PIN_GREEN, CH_G);
  ledcAttachPin(cfg::PIN_BLUE, CH_B);
  write({0, 0, 0});

  prefs_.begin("lamp", false);
  on_ = prefs_.getBool("on", false);
  color_ = prefs_.getUChar("color", 0) % kColors;
  startFade(millis());
}

const char* Lamp::colorName() const { return kPalette[color_].name; }

Lamp::Rgb Lamp::target() const {
  if (!on_) return {0, 0, 0};
  const Swatch& s = kPalette[color_];
  return {s.r / 255.0f, s.g / 255.0f, s.b / 255.0f};
}

void Lamp::toggle() {
  on_ = !on_;
  startFade(millis());
}

void Lamp::nextColor() {
  if (on_) color_ = (color_ + 1) % kColors;
  on_ = true;  // a triple clap on a dark lamp turns it on at its last colour
  startFade(millis());
}

void Lamp::startFade(uint32_t nowMs) {
  from_ = current_;
  fadeStart_ = nowMs;
  fading_ = true;
  dirty_ = true;
  changedAt_ = nowMs;
}

void Lamp::update(uint32_t nowMs) {
  if (fading_) {
    float t = (nowMs - fadeStart_) / static_cast<float>(cfg::FADE_MS);
    if (t >= 1.0f) { t = 1.0f; fading_ = false; }
    const float s = t * t * (3 - 2 * t);  // smoothstep
    const Rgb to = target();
    current_ = {from_.r + (to.r - from_.r) * s, from_.g + (to.g - from_.g) * s,
                from_.b + (to.b - from_.b) * s};
    write(current_);
  }
  if (dirty_ && nowMs - changedAt_ > cfg::SAVE_DELAY_MS) {
    // Deferred so rapid clapping doesn't wear the flash.
    prefs_.putBool("on", on_);
    prefs_.putUChar("color", color_);
    dirty_ = false;
  }
}

void Lamp::write(const Rgb& c) {
  const uint32_t maxDuty = (1u << cfg::PWM_BITS) - 1;
  auto duty = [&](float v) {
    uint32_t d = static_cast<uint32_t>(gamma22(v * cfg::BRIGHTNESS) * maxDuty + 0.5f);
    if (d > maxDuty) d = maxDuty;
    return cfg::LED_COMMON_ANODE ? maxDuty - d : d;
  };
  ledcWrite(CH_R, duty(c.r));
  ledcWrite(CH_G, duty(c.g));
  ledcWrite(CH_B, duty(c.b));
}
