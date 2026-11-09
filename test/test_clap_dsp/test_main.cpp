// Host unit tests for lib/ClapDSP.   pio test -e native
#include <clap_dsp.h>
#include <unity.h>

#include <math.h>

#include <vector>

#include "../../tools/synth.h"

using namespace clap;

void setUp() {}
void tearDown() {}

// ---- high-pass filter ------------------------------------------------------------

void test_highpass_frequency_response() {
  HighPass hp(100.0f, 8000.0f);
  TEST_ASSERT_FLOAT_WITHIN(0.002f, 0.9277f, hp.coefficient());
  TEST_ASSERT_FLOAT_WITHIN(1e-4f, 0.0f, hp.gainAt(0.0f, 8000.0f));            // DC removed
  TEST_ASSERT_FLOAT_WITHIN(0.03f, 0.707f, hp.gainAt(100.0f, 8000.0f));        // -3 dB at fc
  TEST_ASSERT_TRUE(hp.gainAt(50.0f, 8000.0f) < 0.5f);                          // mains hum attenuated
  // Clap band passes. The RC-derived difference equation tops out at
  // 2a/(1+a) = 0.962 at Nyquist instead of exactly 1 - a 0.3 dB loss.
  TEST_ASSERT_TRUE(hp.gainAt(2000.0f, 8000.0f) > 0.95f);
  const float a = hp.coefficient();
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 2 * a / (1 + a), hp.gainAt(4000.0f, 8000.0f));
}

void test_highpass_removes_adc_dc_offset() {
  HighPass hp(100.0f, 8000.0f);
  float y = 0;
  for (int i = 0; i < 800; ++i) y = hp.process(2048.0f + (i == 0 ? 0.0f : 0.0f));
  TEST_ASSERT_FLOAT_WITHIN(1e-3f, 0.0f, y);
  // A sudden 300-count DC step decays to < 1 % within 20 ms (tau ~1.6 ms).
  for (int i = 0; i < 160; ++i) y = hp.process(2348.0f);
  TEST_ASSERT_TRUE(fabsf(y) < 3.0f);
}

void test_highpass_passes_sine_amplitude() {
  HighPass hp(100.0f, 8000.0f);
  float peak = 0;
  for (int i = 0; i < 8000; ++i) {
    const float y = hp.process(2048.0f + 500.0f * sinf(2 * kPi * 1500.0f * i / 8000.0f));
    if (i > 4000) peak = fmaxf(peak, fabsf(y));
  }
  // Measured amplitude matches the analytic |H(1500 Hz)|.
  TEST_ASSERT_FLOAT_WITHIN(5.0f, 500.0f * hp.gainAt(1500.0f, 8000.0f), peak);
}

// ---- FFT ---------------------------------------------------------------------------

void test_fft_sine_lands_in_its_bin() {
  float re[256], im[256];
  for (int i = 0; i < 256; ++i) { re[i] = sinf(2 * kPi * 40 * i / 256.0f); im[i] = 0; }
  fft(re, im, 256);
  int best = 0;
  float bestP = 0;
  for (int k = 0; k <= 128; ++k) {
    const float p = re[k] * re[k] + im[k] * im[k];
    if (p > bestP) { bestP = p; best = k; }
  }
  TEST_ASSERT_EQUAL_INT(40, best);
  TEST_ASSERT_FLOAT_WITHIN(0.5f, 128.0f, sqrtf(bestP));  // N/2 for a unit sine
}

void test_fft_parseval() {
  float re[64], im[64];
  float timeE = 0;
  for (int i = 0; i < 64; ++i) {
    re[i] = cosf(i * 0.7f) + 0.3f * sinf(i * 2.1f);
    im[i] = 0;
    timeE += re[i] * re[i];
  }
  fft(re, im, 64);
  float freqE = 0;
  for (int k = 0; k < 64; ++k) freqE += re[k] * re[k] + im[k] * im[k];
  TEST_ASSERT_FLOAT_WITHIN(timeE * 1e-4f, timeE, freqE / 64.0f);
}

// ---- classifier --------------------------------------------------------------------

Features clapLike() {
  Features f;
  f.riseMs = 0.6f; f.durationMs = 9; f.crest = 8; f.centroidHz = 1800; f.hfRatio = 0.8f;
  return f;
}

void test_classifier_accepts_clap() {
  TEST_ASSERT_TRUE(classify(clapLike(), ClassifierConfig()).isClap);
}

void test_classifier_hard_gates() {
  ClassifierConfig c;
  Features f = clapLike();
  f.riseMs = 15;  // slow onset: voice
  Verdict v = classify(f, c);
  TEST_ASSERT_FALSE(v.isClap);
  TEST_ASSERT_TRUE(v.failed & kFailRise);
  f = clapLike();
  f.durationMs = 220;  // long sound: a word, a note
  v = classify(f, c);
  TEST_ASSERT_FALSE(v.isClap);
  TEST_ASSERT_TRUE(v.failed & kFailLong);
}

void test_classifier_soft_gates_two_of_three() {
  ClassifierConfig c;
  Features f = clapLike();
  f.crest = 2.5f;  // one soft failure is tolerated
  TEST_ASSERT_TRUE(classify(f, c).isClap);
  f.centroidHz = 400;  // two are not (a low thump: door, footstep, kick drum)
  TEST_ASSERT_FALSE(classify(f, c).isClap);
}

// ---- clap-pattern state machine ------------------------------------------------------

void test_fsm_double_clap_toggles_after_timeout() {
  Sequencer s;
  TEST_ASSERT_TRUE(s.onClap(1000) == Command::None);
  TEST_ASSERT_TRUE(s.onClap(1300) == Command::None);
  TEST_ASSERT_TRUE(s.tick(1900) == Command::None);   // still waiting for a 3rd
  TEST_ASSERT_TRUE(s.tick(1951) == Command::Toggle);  // 650 ms after the last clap
  TEST_ASSERT_TRUE(s.state() == Sequencer::State::Lockout);
}

void test_fsm_triple_clap_next_color() {
  Sequencer s;
  s.onClap(0);
  s.onClap(250);
  s.onClap(500);
  TEST_ASSERT_TRUE(s.tick(1200) == Command::NextColor);
}

void test_fsm_single_and_four_claps_do_nothing() {
  Sequencer s;
  s.onClap(0);
  TEST_ASSERT_TRUE(s.tick(2000) == Command::None);
  TEST_ASSERT_TRUE(s.state() == Sequencer::State::Idle);
  for (uint32_t t = 3000; t <= 3900; t += 300) s.onClap(t);  // 4 claps: rhythm, not a command
  TEST_ASSERT_TRUE(s.tick(5000) == Command::None);
}

void test_fsm_echo_within_min_gap_ignored() {
  Sequencer s;
  s.onClap(0);
  s.onClap(40);   // reflection of the same clap
  s.onClap(300);
  TEST_ASSERT_EQUAL_UINT8(2, s.count());
  TEST_ASSERT_TRUE(s.tick(1000) == Command::Toggle);
}

void test_fsm_slow_claps_restart_count() {
  Sequencer s;
  s.onClap(0);
  TEST_ASSERT_TRUE(s.onClap(900) == Command::None);  // too slow: a new pattern starts
  TEST_ASSERT_EQUAL_UINT8(1, s.count());
}

void test_fsm_noise_cancels_pattern() {
  Sequencer s;
  s.onClap(0);
  s.onClap(300);
  s.onNoise(450);  // someone talks
  TEST_ASSERT_TRUE(s.tick(2000) == Command::None);
}

void test_fsm_lockout_ignores_claps() {
  Sequencer s;
  s.onClap(0);
  s.onClap(300);
  TEST_ASSERT_TRUE(s.tick(1000) == Command::Toggle);
  s.onClap(1200);
  s.onClap(1500);
  TEST_ASSERT_TRUE(s.tick(2300) == Command::None);   // both claps were in the 1 s lockout
  TEST_ASSERT_TRUE(s.state() == Sequencer::State::Idle);
}

// ---- end-to-end on synthetic audio -------------------------------------------------------

std::vector<Command> run(const std::vector<uint16_t>& adc) {
  Pipeline p;
  std::vector<Command> out;
  for (uint16_t s : adc) {
    const Command c = p.process(s);
    if (c != Command::None) out.push_back(c);
  }
  for (int i = 0; i < 8000; ++i) {  // 1 s of silence so patterns time out
    const Command c = p.process(adc.back());
    if (c != Command::None) out.push_back(c);
  }
  return out;
}

void test_detector_features_on_clean_clap() {
  synth::Rng r(1);
  synth::Buffer x(synth::ms(400), 0.0f);
  synth::clap(x, synth::ms(100), 1200, r);
  const auto adc = synth::toAdc(x, r);
  HighPass hp(100, 8000);
  Detector d;
  Features f;
  int events = 0;
  for (uint16_t s : adc) {
    if (d.process(hp.process(s), &f)) {
      ++events;
      TEST_ASSERT_TRUE(f.riseMs < 2.0f);
      TEST_ASSERT_TRUE(f.durationMs < 40.0f);
      TEST_ASSERT_TRUE(f.crest > 5.0f);
      TEST_ASSERT_TRUE(f.centroidHz > 1000.0f);
      TEST_ASSERT_TRUE(classify(f, ClassifierConfig()).isClap);
    }
  }
  TEST_ASSERT_EQUAL_INT(1, events);  // reverb tail must not trigger a second event
}

void test_pipeline_double_and_triple() {
  synth::Rng r(42);
  int ok = 0;
  for (int i = 0; i < 20; ++i) {
    const auto d = run(synth::makeClapClip(2, r).adc);
    ok += d.size() == 1 && d[0] == Command::Toggle;
    const auto t = run(synth::makeClapClip(3, r).adc);
    ok += t.size() == 1 && t[0] == Command::NextColor;
  }
  TEST_ASSERT_TRUE_MESSAGE(ok >= 37, "at least 37 of 40 synthetic clap patterns recognised");
}

void test_pipeline_ignores_speech_and_music() {
  synth::Rng r(7);
  int falseTriggers = 0;
  for (int i = 0; i < 30; ++i) {
    falseTriggers += static_cast<int>(run(synth::makeSpeechClip(r).adc).size());
    falseTriggers += static_cast<int>(run(synth::makeMusicClip(r).adc).size());
  }
  TEST_ASSERT_TRUE_MESSAGE(falseTriggers <= 1, "at most 1 false trigger in 60 synthetic clips");
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_highpass_frequency_response);
  RUN_TEST(test_highpass_removes_adc_dc_offset);
  RUN_TEST(test_highpass_passes_sine_amplitude);
  RUN_TEST(test_fft_sine_lands_in_its_bin);
  RUN_TEST(test_fft_parseval);
  RUN_TEST(test_classifier_accepts_clap);
  RUN_TEST(test_classifier_hard_gates);
  RUN_TEST(test_classifier_soft_gates_two_of_three);
  RUN_TEST(test_fsm_double_clap_toggles_after_timeout);
  RUN_TEST(test_fsm_triple_clap_next_color);
  RUN_TEST(test_fsm_single_and_four_claps_do_nothing);
  RUN_TEST(test_fsm_echo_within_min_gap_ignored);
  RUN_TEST(test_fsm_slow_claps_restart_count);
  RUN_TEST(test_fsm_noise_cancels_pattern);
  RUN_TEST(test_fsm_lockout_ignores_claps);
  RUN_TEST(test_detector_features_on_clean_clap);
  RUN_TEST(test_pipeline_double_and_triple);
  RUN_TEST(test_pipeline_ignores_speech_and_music);
  return UNITY_END();
}
