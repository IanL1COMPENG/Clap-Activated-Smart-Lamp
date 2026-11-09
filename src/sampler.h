// 8 kHz microphone sampling paced by a hardware timer interrupt.
//
// The timer ISR only wakes the sampler task (ADC reads on the ESP32 take a
// driver lock and are not ISR-safe). The sampler task runs at high priority,
// reads the 12-bit ADC and pushes the sample into a lock-free single-producer
// / single-consumer ring buffer that the DSP task drains.
#pragma once

#include <Arduino.h>

#include <atomic>

#include "config.h"

class SampleRing {
 public:
  bool push(uint16_t v) {
    const uint32_t h = head_.load(std::memory_order_relaxed);
    if (h - tail_.load(std::memory_order_acquire) >= cfg::RING_SAMPLES) return false;
    buf_[h % cfg::RING_SAMPLES] = v;
    head_.store(h + 1, std::memory_order_release);
    return true;
  }
  bool pop(uint16_t& v) {
    const uint32_t t = tail_.load(std::memory_order_relaxed);
    if (head_.load(std::memory_order_acquire) == t) return false;
    v = buf_[t % cfg::RING_SAMPLES];
    tail_.store(t + 1, std::memory_order_release);
    return true;
  }
  uint32_t size() const { return head_.load() - tail_.load(); }

 private:
  uint16_t buf_[cfg::RING_SAMPLES];
  std::atomic<uint32_t> head_{0}, tail_{0};
};

struct SamplerStats {
  std::atomic<uint32_t> samples{0};
  std::atomic<uint32_t> missedTicks{0};   // timer fired before the last read finished
  std::atomic<uint32_t> ringOverflows{0}; // DSP task fell more than 512 ms behind
};

namespace sampler {
void start(SampleRing* ring, SamplerStats* stats);
}
