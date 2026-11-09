// clap_eval: run the exact firmware pipeline (lib/ClapDSP) over test clips and
// report detection rate and false triggers.
//
// Build:  g++ -std=c++17 -O2 -I lib/ClapDSP/src tools/clap_eval.cpp -o clap_eval
//
// Usage:
//   clap_eval --data DIR [--gain 1500] [--csv events.csv] [-v]
//       DIR/double/*.wav    clips containing one double clap   -> expect TOGGLE
//       DIR/triple/*.wav    clips containing one triple clap   -> expect NEXT_COLOR
//       DIR/negative/*.wav  speech, music, TV, ...             -> expect nothing
//     Any sample rate / channel count of 8/16/24/32-bit PCM or 32-bit float
//     WAV. Clips are mixed to mono, band-limited and resampled to 8 kHz, then
//     scaled to ADC counts (full scale -> +/- gain) around 2048 and
//     quantised to 12 bits, like the microphone module + ESP32 ADC.
//
//   clap_eval --synthetic [--seed N] [--csv events.csv] [-v]
//     Generates 100 clap clips + 200 speech/music clips (tools/synth.h).
//
// Exit code is 0; the numbers are the result.

#include <clap_dsp.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

#include "synth.h"

namespace fs = std::filesystem;
static constexpr double kPiD = 3.14159265358979323846;

struct Result {
  std::string name;
  std::string expected;  // TOGGLE, NEXT_COLOR or NONE
  std::vector<clap::Command> commands;
  int events = 0, claps = 0;
};

static FILE* g_csv = nullptr;
static bool g_verbose = false;

static Result runClip(const std::string& name, const std::string& expected,
                      const std::vector<uint16_t>& adc) {
  clap::Pipeline p;
  Result res{name, expected, {}, 0, 0};
  clap::Event ev;
  bool has = false;
  // Pad 1 s of the clip's own idle level so trailing patterns time out.
  std::vector<uint16_t> x = adc;
  const uint16_t tail = adc.empty() ? 2048 : adc.back();
  x.insert(x.end(), 8000, tail);
  for (uint16_t s : x) {
    const clap::Command c = p.process(s, &ev, &has);
    if (has) {
      ++res.events;
      res.claps += ev.verdict.isClap;
      const auto& f = ev.features;
      if (g_csv)
        fprintf(g_csv, "%s,%s,%.3f,%.1f,%.2f,%.2f,%.2f,%.0f,%.3f,%d,%u\n", name.c_str(),
                expected.c_str(), f.onsetSample / 8000.0, f.peak, f.riseMs, f.durationMs, f.crest,
                f.centroidHz, f.hfRatio, ev.verdict.isClap ? 1 : 0, ev.verdict.failed);
      if (g_verbose)
        printf("    %-28s t=%6.3fs peak=%6.0f rise=%5.2fms dur=%6.1fms crest=%5.2f cent=%5.0fHz hf=%.2f -> %s\n",
               name.c_str(), f.onsetSample / 8000.0, f.peak, f.riseMs, f.durationMs, f.crest,
               f.centroidHz, f.hfRatio, ev.verdict.isClap ? "CLAP" : "-");
    }
    if (c != clap::Command::None) res.commands.push_back(c);
  }
  return res;
}

// ---------------------------------------------------------------------------
// WAV loading
// ---------------------------------------------------------------------------
static bool readWav(const fs::path& path, std::vector<float>& mono, float& rate) {
  std::ifstream f(path, std::ios::binary);
  if (!f) return false;
  std::vector<uint8_t> d((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
  auto u16 = [&](size_t o) { return static_cast<uint32_t>(d[o] | (d[o + 1] << 8)); };
  auto u32 = [&](size_t o) { return u16(o) | (u16(o + 2) << 16); };
  if (d.size() < 12 || memcmp(d.data(), "RIFF", 4) || memcmp(d.data() + 8, "WAVE", 4)) return false;
  uint32_t fmt = 0, ch = 0, bits = 0;
  size_t dataOff = 0, dataLen = 0;
  for (size_t o = 12; o + 8 <= d.size();) {
    const uint32_t len = u32(o + 4);
    if (!memcmp(d.data() + o, "fmt ", 4) && len >= 16) {
      fmt = u16(o + 8);
      ch = u16(o + 10);
      rate = static_cast<float>(u32(o + 12));
      bits = u16(o + 22);
      if (fmt == 0xFFFE && len >= 40) fmt = u16(o + 32);  // WAVE_FORMAT_EXTENSIBLE
    } else if (!memcmp(d.data() + o, "data", 4)) {
      dataOff = o + 8;
      dataLen = std::min<size_t>(len, d.size() - dataOff);
    }
    o += 8 + len + (len & 1);
  }
  if (!dataOff || !ch || !(fmt == 1 || fmt == 3)) return false;
  const size_t bps = bits / 8, frames = dataLen / (bps * ch);
  mono.assign(frames, 0.0f);
  for (size_t i = 0; i < frames; ++i) {
    float acc = 0;
    for (uint32_t c = 0; c < ch; ++c) {
      const uint8_t* p = d.data() + dataOff + (i * ch + c) * bps;
      float v = 0;
      if (fmt == 3 && bits == 32) { memcpy(&v, p, 4); }
      else if (bits == 8) v = (p[0] - 128) / 128.0f;
      else if (bits == 16) v = static_cast<int16_t>(p[0] | (p[1] << 8)) / 32768.0f;
      else if (bits == 24) v = static_cast<int32_t>((p[0] << 8) | (p[1] << 16) | (p[2] << 24)) / 2147483648.0f;
      else if (bits == 32) v = static_cast<int32_t>(p[0] | (p[1] << 8) | (p[2] << 16) | (p[3] << 24)) / 2147483648.0f;
      acc += v;
    }
    mono[i] = acc / ch;
  }
  return true;
}

// Band-limited (windowed-sinc) resampling to 8 kHz.
static std::vector<float> resampleTo8k(const std::vector<float>& x, float rate) {
  if (std::fabs(rate - 8000.0f) < 1.0f) return x;
  const double ratio = rate / 8000.0;
  const double cutoff = std::min(1.0, 8000.0 / rate) * 0.9;  // keep below 3.6 kHz
  const int half = 32;
  const size_t n = static_cast<size_t>(x.size() / ratio);
  std::vector<float> y(n);
  for (size_t i = 0; i < n; ++i) {
    const double t = i * ratio;
    const long c = static_cast<long>(t);
    double acc = 0, norm = 0;
    for (long k = c - half * static_cast<long>(std::ceil(ratio)); k <= c + half * static_cast<long>(std::ceil(ratio)); ++k) {
      if (k < 0 || k >= static_cast<long>(x.size())) continue;
      const double u = (t - k) * cutoff;
      const double sinc = std::fabs(u) < 1e-9 ? 1.0 : std::sin(kPiD * u) / (kPiD * u);
      const double wpos = (t - k) / (half * std::ceil(ratio));
      if (std::fabs(wpos) >= 1) continue;
      const double w = 0.5 + 0.5 * std::cos(kPiD * wpos);
      acc += x[k] * sinc * w;
      norm += sinc * w;
    }
    y[i] = static_cast<float>(norm != 0 ? acc / norm : 0);
  }
  return y;
}

static std::vector<uint16_t> toAdc(const std::vector<float>& x, float gain) {
  std::vector<uint16_t> a(x.size());
  for (size_t i = 0; i < x.size(); ++i)
    a[i] = static_cast<uint16_t>(std::min(4095.0f, std::max(0.0f, std::round(2048.0f + x[i] * gain))));
  return a;
}

// ---------------------------------------------------------------------------

static void report(const std::vector<Result>& results) {
  int pos = 0, posOk = 0, neg = 0, negFalse = 0, falseCmds = 0;
  for (const auto& r : results) {
    std::string got = r.commands.empty() ? "NONE" : clap::commandName(r.commands[0]);
    for (size_t i = 1; i < r.commands.size(); ++i) got += std::string("+") + clap::commandName(r.commands[i]);
    bool ok;
    if (r.expected == "NONE") {
      ++neg;
      ok = r.commands.empty();
      if (!ok) { ++negFalse; falseCmds += static_cast<int>(r.commands.size()); }
    } else {
      ++pos;
      ok = r.commands.size() == 1 && got == r.expected;
      posOk += ok;
    }
    if (!ok || g_verbose)
      printf("  %s %-30s expected %-10s got %-12s (%d events, %d claps)\n", ok ? "ok  " : "FAIL",
             r.name.c_str(), r.expected.c_str(), got.c_str(), r.events, r.claps);
  }
  printf("\n");
  if (pos)
    printf("Clap patterns correctly detected: %d / %d  (%.1f%%)\n", posOk, pos, 100.0 * posOk / pos);
  if (neg)
    printf("False triggers on speech/music:   %d command(s) in %d of %d clips\n", falseCmds, negFalse, neg);
}

int main(int argc, char** argv) {
  std::string dataDir, csvPath;
  bool synthetic = false;
  uint32_t seed = 2050;
  float gain = 1500.0f;
  for (int i = 1; i < argc; ++i) {
    const std::string a = argv[i];
    if (a == "--data" && i + 1 < argc) dataDir = argv[++i];
    else if (a == "--synthetic") synthetic = true;
    else if (a == "--seed" && i + 1 < argc) seed = static_cast<uint32_t>(std::stoul(argv[++i]));
    else if (a == "--gain" && i + 1 < argc) gain = std::stof(argv[++i]);
    else if (a == "--csv" && i + 1 < argc) csvPath = argv[++i];
    else if (a == "-v" || a == "--verbose") g_verbose = true;
    else {
      fprintf(stderr, "usage: clap_eval (--synthetic [--seed N] | --data DIR [--gain G]) [--csv FILE] [-v]\n");
      return 2;
    }
  }
  if (!synthetic && dataDir.empty()) synthetic = true;
  if (!csvPath.empty()) {
    g_csv = fopen(csvPath.c_str(), "w");
    if (g_csv) fprintf(g_csv, "clip,expected,t_s,peak,rise_ms,duration_ms,crest,centroid_hz,hf_ratio,is_clap,failed_mask\n");
  }

  std::vector<Result> results;
  if (synthetic) {
    printf("Synthetic dataset (seed %u): 50 double, 50 triple, 100 speech, 100 music clips\n\n", seed);
    synth::Rng r(seed);
    for (int i = 0; i < 50; ++i) results.push_back(runClip("double_" + std::to_string(i), "TOGGLE", synth::makeClapClip(2, r).adc));
    for (int i = 0; i < 50; ++i) results.push_back(runClip("triple_" + std::to_string(i), "NEXT_COLOR", synth::makeClapClip(3, r).adc));
    for (int i = 0; i < 100; ++i) results.push_back(runClip("speech_" + std::to_string(i), "NONE", synth::makeSpeechClip(r).adc));
    for (int i = 0; i < 100; ++i) results.push_back(runClip("music_" + std::to_string(i), "NONE", synth::makeMusicClip(r).adc));
  } else {
    const std::pair<const char*, const char*> sets[] = {
        {"double", "TOGGLE"}, {"triple", "NEXT_COLOR"}, {"negative", "NONE"}};
    for (const auto& s : sets) {
      const fs::path dir = fs::path(dataDir) / s.first;
      if (!fs::is_directory(dir)) continue;
      std::vector<fs::path> files;
      for (const auto& e : fs::directory_iterator(dir))
        if (e.is_regular_file() && e.path().extension() == ".wav") files.push_back(e.path());
      std::sort(files.begin(), files.end());
      for (const auto& f : files) {
        std::vector<float> mono;
        float rate = 0;
        if (!readWav(f, mono, rate)) {
          fprintf(stderr, "skipping %s (unsupported WAV)\n", f.string().c_str());
          continue;
        }
        const std::string name = std::string(s.first) + "/" + f.filename().string();
        results.push_back(runClip(name, s.second, toAdc(resampleTo8k(mono, rate), gain)));
      }
    }
    printf("Dataset %s: %zu clips\n\n", dataDir.c_str(), results.size());
    if (results.empty()) {
      fprintf(stderr, "no clips found (expected %s/{double,triple,negative}/*.wav)\n", dataDir.c_str());
      return 1;
    }
  }
  report(results);
  if (g_csv) fclose(g_csv);
  return 0;
}
