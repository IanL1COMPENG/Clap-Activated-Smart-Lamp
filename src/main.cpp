// Clap-Activated Smart Lamp
//
//   hardware timer ISR (8 kHz) -> sampler task: 12-bit ADC read -> ring buffer
//   DSP task: high-pass -> onset detector -> features -> classifier -> FSM
//             double clap -> toggle, triple clap -> next colour
//
// Serial (921600 baud) prints every detected sound with its features, which
// is what you use to tune thresholds for your room and microphone.

#include <Arduino.h>
#include <clap_dsp.h>

#include "config.h"
#include "lamp.h"
#include "sampler.h"

namespace {

SampleRing ring;
SamplerStats stats;
Lamp lamp;
clap::Pipeline pipeline;

uint32_t statusLedOffAt = 0;

void logEvent(const clap::Event& ev) {
  const clap::Features& f = ev.features;
  char why[48] = "";
  if (!ev.verdict.isClap) {
    const uint8_t m = ev.verdict.failed;
    snprintf(why, sizeof why, " [%s%s%s%s%s%s]", m & clap::kFailRise ? "rise " : "",
             m & clap::kFailShort ? "short " : "", m & clap::kFailLong ? "long " : "",
             m & clap::kFailCrest ? "crest " : "", m & clap::kFailCentroid ? "centroid " : "",
             m & clap::kFailHf ? "hf" : "");
  }
  Serial.printf("[sound] t=%7.3fs peak=%4.0f rise=%5.2fms dur=%5.1fms crest=%5.2f "
                "centroid=%4.0fHz hf=%.2f -> %s%s\n",
                f.onsetSample / static_cast<float>(cfg::SAMPLE_RATE_HZ), f.peak, f.riseMs,
                f.durationMs, f.crest, f.centroidHz, f.hfRatio,
                ev.verdict.isClap ? "CLAP" : "noise", why);
}

void handleCommand(clap::Command c) {
  if (c == clap::Command::Toggle) lamp.toggle();
  else if (c == clap::Command::NextColor) lamp.nextColor();
  else return;
  Serial.printf("[lamp] %s -> %s, %s\n", clap::commandName(c), lamp.isOn() ? "on" : "off",
                lamp.colorName());
}

// BOOT button: short press toggles, long press (>= 1 s) changes colour.
void pollButton(uint32_t now) {
  static bool wasDown = false;
  static uint32_t downAt = 0;
  const bool down = digitalRead(cfg::PIN_BUTTON) == LOW;
  if (down && !wasDown) downAt = now;
  if (!down && wasDown && now - downAt > 30) {
    handleCommand(now - downAt >= 1000 ? clap::Command::NextColor : clap::Command::Toggle);
  }
  wasDown = down;
}

void dspTask(void*) {
  uint32_t lastStats = millis(), lastSamples = 0;
  uint16_t raw;
  clap::Event ev;
  bool hasEvent = false;

  for (;;) {
    // Drain everything the sampler produced. A feature computation (FFT)
    // takes well under a millisecond; the ring holds 512 ms.
    while (ring.pop(raw)) {
      const clap::Command c = pipeline.process(raw, &ev, &hasEvent);
      if (hasEvent) {
        logEvent(ev);
        if (ev.verdict.isClap) {
          digitalWrite(cfg::PIN_STATUS_LED, HIGH);
          statusLedOffAt = millis() + 60;
        }
      }
      handleCommand(c);
    }

    const uint32_t now = millis();
    if (statusLedOffAt && static_cast<int32_t>(now - statusLedOffAt) >= 0) {
      digitalWrite(cfg::PIN_STATUS_LED, LOW);
      statusLedOffAt = 0;
    }
    pollButton(now);
    lamp.update(now);

    if (now - lastStats >= 10000) {
      const uint32_t s = stats.samples.load();
      Serial.printf("[stats] %.0f samples/s, missed ticks %u, ring overflows %u, noise floor %.1f, "
                    "trigger %.0f\n",
                    (s - lastSamples) * 1000.0f / (now - lastStats), (unsigned)stats.missedTicks.load(),
                    (unsigned)stats.ringOverflows.load(), pipeline.detector().noiseFloor(),
                    pipeline.detector().threshold());
      lastSamples = s;
      lastStats = now;
    }

    vTaskDelay(pdMS_TO_TICKS(4));  // ~32 samples per pass
  }
}

}  // namespace

void setup() {
  Serial.begin(921600);
  delay(100);
  Serial.println("\n[lamp] Clap-Activated Smart Lamp - double clap: on/off, triple clap: colour");

  pinMode(cfg::PIN_STATUS_LED, OUTPUT);
  pinMode(cfg::PIN_BUTTON, INPUT_PULLUP);
  lamp.begin();
  Serial.printf("[lamp] restored: %s, %s\n", lamp.isOn() ? "on" : "off", lamp.colorName());

  xTaskCreatePinnedToCore(dspTask, "dsp", 8192, nullptr, cfg::DSP_PRIORITY, nullptr,
                          cfg::AUDIO_CORE);
  sampler::start(&ring, &stats);
}

void loop() { vTaskDelete(nullptr); }
