// ClapDSP: clap detection and classification for an 8 kHz, 12-bit microphone
// stream. Header-only and hardware-independent, so the ESP32 firmware, the
// unit tests and the PC evaluation tool all run exactly the same code.
//
//   ADC sample -> 1st-order IIR high-pass -> envelope / onset detector
//              -> event capture -> features -> classifier -> clap FSM
#pragma once

#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

namespace clap {

constexpr float kPi = 3.14159265358979f;

// ===========================================================================
// First-order IIR high-pass filter
//   y[n] = a * (y[n-1] + x[n] - x[n-1]),   a = RC / (RC + T),  RC = 1/(2*pi*fc)
// Removes the ADC's DC bias (~2048 counts) and low-frequency rumble / mains
// hum. Pole at z = a, zero at z = 1 (exact zero gain at DC).
// ===========================================================================
class HighPass {
 public:
  HighPass() = default;
  HighPass(float cutoffHz, float sampleRateHz) { configure(cutoffHz, sampleRateHz); }

  void configure(float cutoffHz, float sampleRateHz) {
    const float rc = 1.0f / (2.0f * kPi * cutoffHz);
    const float dt = 1.0f / sampleRateHz;
    a_ = rc / (rc + dt);
    primed_ = false;
  }

  float process(float x) {
    if (!primed_) {  // start settled on the first sample's DC level
      xPrev_ = x;
      yPrev_ = 0.0f;
      primed_ = true;
      return 0.0f;
    }
    const float y = a_ * (yPrev_ + x - xPrev_);
    xPrev_ = x;
    yPrev_ = y;
    return y;
  }

  float coefficient() const { return a_; }

  // |H(f)| of this filter, for tests and documentation.
  float gainAt(float hz, float sampleRateHz) const {
    const float w = 2.0f * kPi * hz / sampleRateHz;
    // H(z) = a (1 - z^-1) / (1 - a z^-1)
    const float nr = 1.0f - cosf(w), ni = sinf(w);
    const float dr = 1.0f - a_ * cosf(w), di = a_ * sinf(w);
    return a_ * sqrtf((nr * nr + ni * ni) / (dr * dr + di * di));
  }

 private:
  float a_ = 0.0f;
  float xPrev_ = 0.0f;
  float yPrev_ = 0.0f;
  bool primed_ = false;
};

// ===========================================================================
// In-place iterative radix-2 complex FFT (n must be a power of two)
// ===========================================================================
inline void fft(float* re, float* im, size_t n) {
  // bit-reversal permutation
  for (size_t i = 1, j = 0; i < n; ++i) {
    size_t bit = n >> 1;
    for (; j & bit; bit >>= 1) j ^= bit;
    j ^= bit;
    if (i < j) {
      float t = re[i]; re[i] = re[j]; re[j] = t;
      t = im[i]; im[i] = im[j]; im[j] = t;
    }
  }
  for (size_t len = 2; len <= n; len <<= 1) {
    const float ang = -2.0f * kPi / static_cast<float>(len);
    const float wr = cosf(ang), wi = sinf(ang);
    for (size_t i = 0; i < n; i += len) {
      float cr = 1.0f, ci = 0.0f;
      for (size_t k = 0; k < len / 2; ++k) {
        const size_t a = i + k, b = i + k + len / 2;
        const float tr = re[b] * cr - im[b] * ci;
        const float ti = re[b] * ci + im[b] * cr;
        re[b] = re[a] - tr; im[b] = im[a] - ti;
        re[a] += tr;        im[a] += ti;
        const float ncr = cr * wr - ci * wi;
        ci = cr * wi + ci * wr;
        cr = ncr;
      }
    }
  }
}

// ===========================================================================
// Features and classifier
// ===========================================================================
struct Features {
  uint32_t onsetSample = 0;  // sample index where the envelope crossed the trigger
  float peak = 0;            // peak |x| after the high-pass (ADC counts)
  float riseMs = 0;          // trigger crossing -> envelope peak
  float durationMs = 0;      // trigger crossing -> envelope below endFraction * peak
  float crest = 0;           // peak / RMS over the capture window
  float centroidHz = 0;      // spectral centroid of the first 32 ms
  float hfRatio = 0;         // energy 1-4 kHz / energy 150 Hz-4 kHz
  float noiseFloor = 0;      // envelope noise floor when the event started
};

struct ClassifierConfig {
  // Hard gates: a clap is an impulse.
  float maxRiseMs = 3.0f;
  float minDurationMs = 1.0f;
  float maxDurationMs = 90.0f;
  // Soft gates: at least `minSoftPasses` of these must agree.
  float minCrest = 4.0f;
  float minCentroidHz = 1000.0f;
  float minHfRatio = 0.45f;
  uint8_t minSoftPasses = 2;
};

enum FailBits : uint8_t {
  kFailRise = 1 << 0,
  kFailShort = 1 << 1,
  kFailLong = 1 << 2,
  kFailCrest = 1 << 3,
  kFailCentroid = 1 << 4,
  kFailHf = 1 << 5,
};

struct Verdict {
  bool isClap = false;
  uint8_t failed = 0;  // FailBits
};

inline Verdict classify(const Features& f, const ClassifierConfig& c) {
  Verdict v;
  if (f.riseMs > c.maxRiseMs) v.failed |= kFailRise;
  if (f.durationMs < c.minDurationMs) v.failed |= kFailShort;
  if (f.durationMs > c.maxDurationMs) v.failed |= kFailLong;
  uint8_t soft = 0;
  if (f.crest >= c.minCrest) ++soft; else v.failed |= kFailCrest;
  if (f.centroidHz >= c.minCentroidHz) ++soft; else v.failed |= kFailCentroid;
  if (f.hfRatio >= c.minHfRatio) ++soft; else v.failed |= kFailHf;
  const bool hardOk = (v.failed & (kFailRise | kFailShort | kFailLong)) == 0;
  v.isClap = hardOk && soft >= c.minSoftPasses;
  return v;
}

// ===========================================================================
// Onset detector + event capture + feature extraction
// ===========================================================================
struct DetectorConfig {
  float sampleRateHz = 8000.0f;
  float triggerRatio = 8.0f;      // envelope must exceed noise floor x this (18 dB)
  float minTrigger = 100.0f;      // ...and this absolute level (ADC counts)
  float releaseMs = 4.0f;         // envelope release time constant
  float floorRiseMs = 2000.0f;    // noise floor follows rises slowly (ignores claps)
  float floorFallMs = 150.0f;     // ...and drops quickly when it gets quiet
  float endFraction = 0.25f;      // event ends at -12 dB from its peak...
  float holdMs = 25.0f;           // ...once it has stayed there this long
  float maxEventMs = 300.0f;      // give up timing longer sounds
  float refractoryMs = 50.0f;     // ignore the clap's own reverb tail
};

class Detector {
 public:
  static constexpr size_t kPre = 32;      // 4 ms before the onset
  static constexpr size_t kPost = 512;    // 64 ms after
  static constexpr size_t kCapture = kPre + kPost;
  static constexpr size_t kFft = 256;     // 32 ms analysis window

  explicit Detector(const DetectorConfig& cfg = DetectorConfig()) { configure(cfg); }

  void configure(const DetectorConfig& cfg) {
    cfg_ = cfg;
    const float fs = cfg.sampleRateHz;
    release_ = expf(-1.0f / (cfg.releaseMs * 1e-3f * fs));
    floorUp_ = 1.0f - expf(-1.0f / (cfg.floorRiseMs * 1e-3f * fs));
    floorDown_ = 1.0f - expf(-1.0f / (cfg.floorFallMs * 1e-3f * fs));
    maxEventSamples_ = static_cast<uint32_t>(cfg.maxEventMs * 1e-3f * fs);
    refractorySamples_ = static_cast<uint32_t>(cfg.refractoryMs * 1e-3f * fs);
    holdSamples_ = static_cast<uint32_t>(cfg.holdMs * 1e-3f * fs);
    reset();
  }

  void reset() {
    state_ = State::Idle;
    env_ = 0;
    floor_ = cfg_.minTrigger / cfg_.triggerRatio;
    preHead_ = 0;
    armed_ = true;
    memset(pre_, 0, sizeof pre_);
    n_ = 0;
  }

  float envelope() const { return env_; }
  float noiseFloor() const { return floor_; }
  float threshold() const {
    const float t = floor_ * cfg_.triggerRatio;
    return t > cfg_.minTrigger ? t : cfg_.minTrigger;
  }
  bool busy() const { return state_ == State::Capture; }

  // Feed one high-passed sample. Returns true when an event has been fully
  // captured; its features are written to *out.
  bool process(float x, Features* out) {
    const uint32_t idx = n_++;
    const float a = fabsf(x);
    env_ = a > env_ ? a : env_ * release_;

    // Noise floor: asymmetric smoothing so short transients barely move it.
    floor_ += (env_ - floor_) * (env_ > floor_ ? floorUp_ : floorDown_);

    switch (state_) {
      case State::Idle:
        // Re-arm only once the envelope has dropped back below the trigger,
        // so a clap's reverb tail can't start a second event.
        if (!armed_ && env_ < 0.7f * threshold()) armed_ = true;
        if (armed_ && env_ > threshold()) {
          startCapture(idx, x);
        } else {
          pre_[preHead_] = x;
          preHead_ = (preHead_ + 1) % kPre;
        }
        return false;

      case State::Capture: {
        if (capLen_ < kCapture) cap_[capLen_++] = x;
        const uint32_t rel = idx - onset_;
        if (!timed_) {
          if (env_ > peakEnv_) {
            peakEnv_ = env_;
            peakAt_ = rel;
          }
          // The sound has ended once the envelope has *stayed* below
          // endFraction of its peak for holdMs. A plosive ("t", "k") dips
          // briefly before its vowel; the hold keeps the vowel in the event.
          if (rel > peakAt_ && env_ < cfg_.endFraction * peakEnv_) {
            if (belowRun_++ == 0) belowStart_ = rel;
          } else {
            belowRun_ = 0;
          }
          if (belowRun_ >= holdSamples_) {
            timed_ = true;
            durationSamples_ = belowStart_;
          } else if (rel >= maxEventSamples_) {
            timed_ = true;
            durationSamples_ = rel;
          }
        }
        if (timed_ && capLen_ >= kCapture) {
          if (out) computeFeatures(out);
          state_ = State::Refractory;
          refractoryLeft_ = refractorySamples_;
          return true;
        }
        return false;
      }

      case State::Refractory:
        pre_[preHead_] = x;
        preHead_ = (preHead_ + 1) % kPre;
        if (refractoryLeft_ == 0 || --refractoryLeft_ == 0) state_ = State::Idle;
        return false;
    }
    return false;
  }

 private:
  enum class State : uint8_t { Idle, Capture, Refractory };

  void startCapture(uint32_t idx, float x) {
    for (size_t i = 0; i < kPre; ++i) cap_[i] = pre_[(preHead_ + i) % kPre];
    capLen_ = kPre;
    cap_[capLen_++] = x;
    onset_ = idx;
    floorAtOnset_ = floor_;
    peakEnv_ = env_;
    peakAt_ = 0;
    timed_ = false;
    durationSamples_ = 0;
    belowRun_ = 0;
    belowStart_ = 0;
    state_ = State::Capture;
    armed_ = false;
  }

  void computeFeatures(Features* f) {
    const float msPerSample = 1000.0f / cfg_.sampleRateHz;
    f->onsetSample = onset_;
    f->noiseFloor = floorAtOnset_;
    f->durationMs = durationSamples_ * msPerSample;

    // 10-80 % rise time on a fast envelope (instant attack, 2 ms release).
    // A clap gets from 10 % to 90 % of its peak in about a millisecond;
    // voiced sounds take 10+ ms.
    const float rel1ms = expf(-1.0f / (2e-3f * cfg_.sampleRateHz));
    float e = 0;
    int last10 = -1, first90 = -1;
    for (size_t i = 0; i < kCapture; ++i) {
      const float a = fabsf(cap_[i]);
      e = a > e ? a : e * rel1ms;
      if (e < 0.1f * peakEnv_) last10 = static_cast<int>(i);
      if (e >= 0.8f * peakEnv_) { first90 = static_cast<int>(i); break; }
    }
    if (first90 < 0) {
      f->riseMs = peakAt_ * msPerSample;  // peak came after the capture window
    } else {
      f->riseMs = (first90 - last10) * msPerSample;
    }

    // Crest factor over the whole capture window.
    float peak = 0, sumSq = 0;
    for (size_t i = 0; i < kCapture; ++i) {
      const float v = cap_[i];
      const float av = fabsf(v);
      if (av > peak) peak = av;
      sumSq += v * v;
    }
    const float rms = sqrtf(sumSq / kCapture);
    f->peak = peak;
    f->crest = rms > 0 ? peak / rms : 0;

    // Spectrum of the first 32 ms, starting 1 ms before the onset.
    const size_t start = kPre - 8;
    for (size_t i = 0; i < kFft; ++i) {
      const float w = 0.5f - 0.5f * cosf(2.0f * kPi * i / (kFft - 1));  // Hann
      re_[i] = cap_[start + i] * w;
      im_[i] = 0;
    }
    fft(re_, im_, kFft);
    const float binHz = cfg_.sampleRateHz / kFft;
    const size_t lo = static_cast<size_t>(150.0f / binHz + 0.5f);
    const size_t hfLo = static_cast<size_t>(1000.0f / binHz + 0.5f);
    float total = 0, hf = 0, weighted = 0;
    for (size_t k = lo; k <= kFft / 2; ++k) {
      const float p = re_[k] * re_[k] + im_[k] * im_[k];
      total += p;
      weighted += p * (k * binHz);
      if (k >= hfLo) hf += p;
    }
    f->centroidHz = total > 0 ? weighted / total : 0;
    f->hfRatio = total > 0 ? hf / total : 0;
  }

  DetectorConfig cfg_;
  float release_ = 0, floorUp_ = 0, floorDown_ = 0;
  uint32_t maxEventSamples_ = 0, refractorySamples_ = 0;

  State state_ = State::Idle;
  uint32_t n_ = 0;
  float env_ = 0, floor_ = 0;

  float pre_[kPre];
  size_t preHead_ = 0;
  float cap_[kCapture];
  size_t capLen_ = 0;

  uint32_t onset_ = 0, peakAt_ = 0, durationSamples_ = 0, refractoryLeft_ = 0;
  uint32_t holdSamples_ = 0, belowRun_ = 0, belowStart_ = 0;
  float peakEnv_ = 0, floorAtOnset_ = 0;
  bool timed_ = false;
  bool armed_ = true;

  float re_[kFft], im_[kFft];
};

// ===========================================================================
// Clap-pattern finite state machine
//
//   IDLE --clap--> COUNTING --clap within [minGap, maxGap]--> COUNTING (n+1)
//   COUNTING --timeout (no clap for maxGap)--> n==2: TOGGLE    -> LOCKOUT
//                                              n==3: NEXT_COLOR -> LOCKOUT
//                                              otherwise         -> IDLE
//   COUNTING --loud non-clap sound--> IDLE      (speech/music cancels)
//   COUNTING --4th clap--> keeps counting, fires nothing (rhythmic noise)
//   LOCKOUT --lockoutMs elapsed--> IDLE         (claps ignored meanwhile)
// ===========================================================================
enum class Command : uint8_t { None, Toggle, NextColor };

inline const char* commandName(Command c) {
  switch (c) {
    case Command::Toggle: return "TOGGLE";
    case Command::NextColor: return "NEXT_COLOR";
    default: return "NONE";
  }
}

struct SequenceConfig {
  uint32_t minGapMs = 100;    // closer than this = the same clap's echo
  uint32_t maxGapMs = 650;    // longer than this ends the pattern
  uint32_t lockoutMs = 1000;  // after a command, ignore everything
};

class Sequencer {
 public:
  enum class State : uint8_t { Idle, Counting, Lockout };

  explicit Sequencer(const SequenceConfig& cfg = SequenceConfig()) : cfg_(cfg) {}

  State state() const { return state_; }
  uint8_t count() const { return count_; }

  Command onClap(uint32_t tMs) {
    Command c = tick(tMs);
    if (c != Command::None) return c;
    switch (state_) {
      case State::Idle:
        state_ = State::Counting;
        count_ = 1;
        last_ = tMs;
        break;
      case State::Counting:
        if (tMs - last_ < cfg_.minGapMs) break;  // echo / double detection
        if (count_ < 255) ++count_;
        last_ = tMs;
        break;
      case State::Lockout:
        break;
    }
    return Command::None;
  }

  void onNoise(uint32_t tMs) {
    tick(tMs);
    if (state_ == State::Counting) {
      state_ = State::Idle;
      count_ = 0;
    }
  }

  Command tick(uint32_t tMs) {
    if (state_ == State::Counting && tMs - last_ > cfg_.maxGapMs) {
      Command c = Command::None;
      if (count_ == 2) c = Command::Toggle;
      else if (count_ == 3) c = Command::NextColor;
      count_ = 0;
      if (c != Command::None) {
        state_ = State::Lockout;
        lockStart_ = tMs;
      } else {
        state_ = State::Idle;
      }
      return c;
    }
    if (state_ == State::Lockout && static_cast<int32_t>(tMs - lockStart_) >= static_cast<int32_t>(cfg_.lockoutMs))
      state_ = State::Idle;
    return Command::None;
  }

 private:
  SequenceConfig cfg_;
  State state_ = State::Idle;
  uint8_t count_ = 0;
  uint32_t last_ = 0;
  uint32_t lockStart_ = 0;
};

// ===========================================================================
// Full pipeline: raw 12-bit ADC samples in, lamp commands out
// ===========================================================================
struct PipelineConfig {
  float sampleRateHz = 8000.0f;
  float highPassHz = 100.0f;
  DetectorConfig detector;
  ClassifierConfig classifier;
  SequenceConfig sequence;
};

struct Event {
  Features features;
  Verdict verdict;
};

class Pipeline {
 public:
  explicit Pipeline(const PipelineConfig& cfg = PipelineConfig())
      : cfg_(cfg), hp_(cfg.highPassHz, cfg.sampleRateHz), det_(cfg.detector), seq_(cfg.sequence) {}

  // Process one ADC sample. Returns a command when a clap pattern completes.
  // If an event (clap or not) was classified on this sample, *event is
  // filled and *hasEvent set.
  Command process(uint16_t adcRaw, Event* event = nullptr, bool* hasEvent = nullptr) {
    const uint32_t idx = sample_++;
    const uint32_t tMs = static_cast<uint32_t>(static_cast<uint64_t>(idx) * 1000u /
                                               static_cast<uint32_t>(cfg_.sampleRateHz));
    if (hasEvent) *hasEvent = false;
    const float y = hp_.process(static_cast<float>(adcRaw));

    Features f;
    if (det_.process(y, &f)) {
      const Verdict v = classify(f, cfg_.classifier);
      if (event) {
        event->features = f;
        event->verdict = v;
      }
      if (hasEvent) *hasEvent = true;
      // Timestamp the clap at its onset, not when its analysis finished.
      const uint32_t onsetMs = static_cast<uint32_t>(static_cast<uint64_t>(f.onsetSample) * 1000u /
                                                     static_cast<uint32_t>(cfg_.sampleRateHz));
      if (v.isClap) return seq_.onClap(onsetMs);
      seq_.onNoise(onsetMs);
      return Command::None;
    }
    // While an event is still being captured its clap hasn't been counted
    // yet, so don't let the pattern time out underneath it.
    if (!det_.busy() && (idx & 7u) == 0) return seq_.tick(tMs);
    return Command::None;
  }

  uint32_t samples() const { return sample_; }
  const Detector& detector() const { return det_; }
  const Sequencer& sequencer() const { return seq_; }

 private:
  PipelineConfig cfg_;
  HighPass hp_;
  Detector det_;
  Sequencer seq_;
  uint32_t sample_ = 0;
};

}  // namespace clap
