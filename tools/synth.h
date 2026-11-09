// Synthetic 8 kHz test audio for the clap classifier: claps with room reverb,
// formant-synthesised speech, and music (tonal, percussive, pads).
//
// This is a regression harness, not a substitute for real recordings: it
// checks that the pipeline behaves sensibly on signals with the right
// physical character (impulse vs. voiced vs. tonal). Real-world numbers come
// from running tools/clap_eval on recorded WAV clips.
#pragma once

#include <math.h>
#include <stdint.h>

#include <algorithm>
#include <vector>

namespace synth {

constexpr float kFs = 8000.0f;
constexpr float kPi = 3.14159265358979f;

class Rng {
 public:
  explicit Rng(uint32_t seed) : s_(seed ? seed : 0x9E3779B9u) {}
  uint32_t next() {
    s_ ^= s_ << 13;
    s_ ^= s_ >> 17;
    s_ ^= s_ << 5;
    return s_;
  }
  float uni() { return (next() >> 8) * (1.0f / 16777216.0f); }
  float uni(float a, float b) { return a + (b - a) * uni(); }
  bool chance(float p) { return uni() < p; }
  float gauss() {
    const float u1 = std::max(uni(), 1e-7f), u2 = uni();
    return sqrtf(-2.0f * logf(u1)) * cosf(2.0f * kPi * u2);
  }

 private:
  uint32_t s_;
};

// RBJ biquad
struct Biquad {
  float b0 = 1, b1 = 0, b2 = 0, a1 = 0, a2 = 0, z1 = 0, z2 = 0;
  static Biquad bandpass(float fc, float q) {
    Biquad f;
    const float w = 2 * kPi * fc / kFs, al = sinf(w) / (2 * q), a0 = 1 + al;
    f.b0 = al / a0; f.b1 = 0; f.b2 = -al / a0;
    f.a1 = -2 * cosf(w) / a0; f.a2 = (1 - al) / a0;
    return f;
  }
  static Biquad highpass(float fc, float q = 0.707f) {
    Biquad f;
    const float w = 2 * kPi * fc / kFs, al = sinf(w) / (2 * q), c = cosf(w), a0 = 1 + al;
    f.b0 = (1 + c) / 2 / a0; f.b1 = -(1 + c) / a0; f.b2 = (1 + c) / 2 / a0;
    f.a1 = -2 * c / a0; f.a2 = (1 - al) / a0;
    return f;
  }
  static Biquad lowpass(float fc, float q = 0.707f) {
    Biquad f;
    const float w = 2 * kPi * fc / kFs, al = sinf(w) / (2 * q), c = cosf(w), a0 = 1 + al;
    f.b0 = (1 - c) / 2 / a0; f.b1 = (1 - c) / a0; f.b2 = (1 - c) / 2 / a0;
    f.a1 = -2 * c / a0; f.a2 = (1 - al) / a0;
    return f;
  }
  float operator()(float x) {
    const float y = b0 * x + z1;
    z1 = b1 * x - a1 * y + z2;
    z2 = b2 * x - a2 * y;
    return y;
  }
};

using Buffer = std::vector<float>;

inline void addScaled(Buffer& out, size_t at, const Buffer& src, float targetPeak) {
  float pk = 0;
  for (float v : src) pk = std::max(pk, fabsf(v));
  if (pk <= 0) return;
  const float g = targetPeak / pk;
  for (size_t i = 0; i < src.size() && at + i < out.size(); ++i) out[at + i] += src[i] * g;
}

inline size_t ms(float m) { return static_cast<size_t>(m * kFs / 1000.0f); }

// ---------------------------------------------------------------------------
// Hand clap: ~1 ms attack, few-ms decay, band-limited noise centred 1-2.5 kHz,
// followed by a diffuse room-reverb tail.
// ---------------------------------------------------------------------------
inline void clap(Buffer& out, size_t at, float peak, Rng& r) {
  const float attack = r.uni(0.2f, 0.9f), decay = r.uni(2.0f, 7.0f);
  Biquad bp = Biquad::bandpass(r.uni(1100, 2600), r.uni(0.7f, 1.4f));
  Buffer burst(ms(45));
  for (size_t i = 0; i < burst.size(); ++i) {
    const float t = i * 1000.0f / kFs;
    const float env = t < attack ? t / attack : expf(-(t - attack) / decay);
    const float n = r.gauss();
    burst[i] = (0.8f * bp(n) * 3.0f + 0.2f * n) * env;
  }
  addScaled(out, at, burst, peak);

  const float rt60 = r.uni(0.2f, 0.6f), tau = rt60 / 6.91f * 1000.0f;
  const float level = peak * r.uni(0.03f, 0.10f);
  Biquad lp = Biquad::lowpass(r.uni(2500, 3800));
  const size_t start = at + ms(r.uni(2, 6)), len = ms(3 * tau);
  for (size_t i = 0; i < len && start + i < out.size(); ++i) {
    const float t = i * 1000.0f / kFs;
    const float env = std::min(1.0f, t / 3.0f) * expf(-t / tau);
    out[start + i] += lp(r.gauss()) * level * env;
  }
}

// ---------------------------------------------------------------------------
// Speech: voiced syllables (glottal pulse train through 3 formants) with
// optional plosive / fricative onsets.
// ---------------------------------------------------------------------------
inline Buffer vowel(float durMs, float f0, Rng& r, float attackMs, float releaseMs) {
  Buffer v(ms(durMs));
  Biquad f1 = Biquad::bandpass(r.uni(300, 800), r.uni(4, 8));
  Biquad f2 = Biquad::bandpass(r.uni(900, 2300), r.uni(5, 10));
  Biquad f3 = Biquad::bandpass(r.uni(2300, 3200), r.uni(6, 12));
  Biquad glottal = Biquad::lowpass(r.uni(500, 900));
  const float g2 = r.uni(0.3f, 0.7f), g3 = r.uni(0.1f, 0.3f);
  float phase = 0;
  const float drift = r.uni(-0.25f, 0.25f);
  for (size_t i = 0; i < v.size(); ++i) {
    const float t = i * 1000.0f / kFs;
    const float f = f0 * (1.0f + drift * t / durMs);
    phase += f / kFs;
    float src = 0;
    if (phase >= 1.0f) { phase -= 1.0f; src = 1.0f; }
    src = glottal(src) + 0.02f * r.gauss();
    float env = 1.0f;
    if (t < attackMs) env = 0.5f - 0.5f * cosf(kPi * t / attackMs);
    else if (t > durMs - releaseMs) env = 0.5f + 0.5f * cosf(kPi * (t - (durMs - releaseMs)) / releaseMs);
    v[i] = (f1(src) + g2 * f2(src) + g3 * f3(src)) * env;
  }
  return v;
}

inline Buffer noiseBurst(float durMs, float attackMs, float decayMs, Biquad filt, Rng& r) {
  Buffer b(ms(durMs));
  for (size_t i = 0; i < b.size(); ++i) {
    const float t = i * 1000.0f / kFs;
    const float env = t < attackMs ? t / attackMs : expf(-(t - attackMs) / decayMs);
    b[i] = filt(r.gauss()) * env;
  }
  return b;
}

inline void speech(Buffer& out, size_t from, size_t to, float level, Rng& r) {
  size_t pos = from + ms(r.uni(0, 150));
  const float f0 = r.uni(90, 230);
  while (pos < to) {
    const float a = level * r.uni(0.5f, 1.0f);
    const float kind = r.uni();
    if (kind < 0.35f) {  // plosive: short broadband burst, then the vowel
      addScaled(out, pos, noiseBurst(12, 0.5f, 4, Biquad::bandpass(r.uni(1500, 3500), 1), r),
                a * r.uni(0.2f, 0.6f));
      pos += ms(r.uni(10, 35));
    } else if (kind < 0.55f) {  // fricative
      const float d = r.uni(60, 160);
      addScaled(out, pos, noiseBurst(d, r.uni(10, 30), d / 3, Biquad::highpass(r.uni(1800, 3000)), r),
                a * r.uni(0.15f, 0.4f));
      pos += ms(d * 0.7f);
    }
    const float dur = r.uni(80, 280);
    addScaled(out, pos, vowel(dur, f0 * r.uni(0.85f, 1.2f), r, r.uni(12, 40), r.uni(25, 70)), a);
    pos += ms(dur + r.uni(30, 220));
  }
}

// ---------------------------------------------------------------------------
// Music
// ---------------------------------------------------------------------------
inline Buffer tone(float durMs, float hz, float attackMs, Rng& r) {
  Buffer b(ms(durMs));
  const int harmonics = 1 + static_cast<int>(r.uni(2, 6));
  const float decayTau = r.uni(150, 900);
  for (size_t i = 0; i < b.size(); ++i) {
    const float t = i * 1000.0f / kFs;
    float env = t < attackMs ? t / attackMs : 0.35f + 0.65f * expf(-(t - attackMs) / decayTau);
    if (t > durMs - 30) env *= std::max(0.0f, (durMs - t) / 30.0f);
    float s = 0;
    for (int h = 1; h <= harmonics; ++h) {
      if (hz * h > 3900) break;
      s += sinf(2 * kPi * hz * h * i / kFs) / h;
    }
    b[i] = s * env;
  }
  return b;
}

inline float noteHz(int semitoneFromA3) { return 220.0f * powf(2.0f, semitoneFromA3 / 12.0f); }

inline void music(Buffer& out, size_t from, size_t to, float level, Rng& r) {
  const float style = r.uni();
  const bool drums = style < 0.35f;
  const bool pad = style > 0.8f;
  // Harmony / melody
  size_t pos = from;
  const int root = static_cast<int>(r.uni(-5, 7));
  while (pos < to) {
    const float dur = pad ? r.uni(600, 1500) : r.uni(150, 600);
    const float attack = pad ? r.uni(80, 250) : (r.chance(0.4f) ? r.uni(2, 8) : r.uni(10, 40));
    const int voices = 1 + static_cast<int>(r.uni(0, 3));
    static const int scale[] = {0, 2, 4, 5, 7, 9, 11, 12, 14, 16};
    for (int v = 0; v < voices; ++v) {
      const int deg = scale[static_cast<int>(r.uni(0, 10)) % 10];
      addScaled(out, pos, tone(dur, noteHz(root + deg + (v ? -12 : 0)), attack, r),
                level * r.uni(0.25f, 0.6f));
    }
    pos += ms(dur * r.uni(0.8f, 1.0f));
  }
  if (r.chance(0.3f)) speech(out, from, to, level * 0.7f, r);  // vocals
  if (!drums) return;
  const float beatMs = 60000.0f / r.uni(85, 145);
  int step = 0;
  for (size_t p = from; p < to; p += ms(beatMs / 2), ++step) {
    if (step % 4 == 0) {  // kick
      Buffer k(ms(150));
      float ph = 0;
      for (size_t i = 0; i < k.size(); ++i) {
        const float t = i * 1000.0f / kFs;
        ph += (55 + 90 * expf(-t / 25)) / kFs;
        k[i] = sinf(2 * kPi * ph) * expf(-t / 60) * std::min(1.0f, t / 1.0f);
      }
      addScaled(out, p, k, level * 0.9f);
    }
    if (step % 4 == 2)  // snare
      addScaled(out, p, noiseBurst(120, 1, r.uni(30, 70), Biquad::bandpass(r.uni(1200, 2500), 0.8f), r),
                level * r.uni(0.5f, 0.9f));
    if (r.chance(0.8f))  // hi-hat on eighths
      addScaled(out, p, noiseBurst(50, 0.5f, r.uni(8, 25), Biquad::highpass(3000), r),
                level * r.uni(0.15f, 0.35f));
  }
}

// ---------------------------------------------------------------------------
// Microphone module + ADC model: DC bias, hum, noise, 12-bit quantisation
// ---------------------------------------------------------------------------
inline std::vector<uint16_t> toAdc(const Buffer& x, Rng& r) {
  const float dc = 2048.0f + r.uni(-200, 200);
  const float hum = r.uni(0, 40), humHz = r.chance(0.5f) ? 50.0f : 60.0f;
  const float noise = r.uni(3, 12);
  std::vector<uint16_t> adc(x.size());
  for (size_t i = 0; i < x.size(); ++i) {
    float v = dc + x[i] + hum * sinf(2 * kPi * humHz * i / kFs) + noise * r.gauss();
    v = std::min(4095.0f, std::max(0.0f, roundf(v)));
    adc[i] = static_cast<uint16_t>(v);
  }
  return adc;
}

enum class Label { Double, Triple, Speech, Music };

struct Clip {
  Label label;
  std::vector<uint16_t> adc;
};

inline Clip makeClapClip(int claps, Rng& r) {
  Buffer x(ms(3000), 0.0f);
  size_t pos = ms(r.uni(200, 700));
  const float level = r.uni(350, 1700);
  for (int i = 0; i < claps; ++i) {
    clap(x, pos, level * r.uni(0.75f, 1.0f), r);
    pos += ms(r.uni(180, 480));
  }
  if (r.chance(0.25f)) speech(x, 0, x.size(), level * r.uni(0.04f, 0.10f), r);  // quiet talk nearby
  return {claps == 2 ? Label::Double : Label::Triple, toAdc(x, r)};
}

inline Clip makeSpeechClip(Rng& r) {
  Buffer x(ms(4000), 0.0f);
  speech(x, 0, x.size(), r.uni(250, 1300), r);
  return {Label::Speech, toAdc(x, r)};
}

inline Clip makeMusicClip(Rng& r) {
  Buffer x(ms(4000), 0.0f);
  music(x, 0, x.size(), r.uni(200, 1000), r);
  return {Label::Music, toAdc(x, r)};
}

}  // namespace synth
