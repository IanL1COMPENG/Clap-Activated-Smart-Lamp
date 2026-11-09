// Pin map and tuning for the clap lamp. Classifier thresholds live in
// lib/ClapDSP/src/clap_dsp.h (ClassifierConfig / DetectorConfig /
// SequenceConfig) so the PC evaluation tool uses the same values.
#pragma once

#include <stdint.h>

namespace cfg {

// ---- Microphone -------------------------------------------------------------
// Analog mic module (MAX9814 / MAX4466 / KY-037 AO) on an ADC1 pin.
// ADC2 can't be used: it is shared with Wi-Fi.
constexpr uint8_t PIN_MIC = 34;              // GPIO34 = ADC1_CHANNEL_6
constexpr uint32_t SAMPLE_RATE_HZ = 8000;    // 125 us per sample
constexpr uint8_t SAMPLE_TIMER = 0;          // hardware timer group 0, timer 0
constexpr uint16_t TIMER_PRESCALER = 80;     // 80 MHz APB / 80 = 1 MHz tick

// ---- RGB LED ------------------------------------------------------------------
// Common-cathode RGB LED (via resistors) or an RGB strip through 3 logic-level
// N-MOSFETs. Set LED_COMMON_ANODE for a common-anode LED.
constexpr uint8_t PIN_RED = 25;
constexpr uint8_t PIN_GREEN = 26;
constexpr uint8_t PIN_BLUE = 27;
constexpr bool LED_COMMON_ANODE = false;
constexpr uint32_t PWM_FREQ_HZ = 5000;
constexpr uint8_t PWM_BITS = 12;
constexpr uint32_t FADE_MS = 350;
constexpr float BRIGHTNESS = 0.85f;

// ---- Misc ------------------------------------------------------------------------
constexpr uint8_t PIN_STATUS_LED = 2;    // on-board LED blinks on every detected clap
constexpr uint8_t PIN_BUTTON = 0;        // BOOT button: manual toggle, hold 1 s = next colour
constexpr uint32_t SAVE_DELAY_MS = 3000; // write lamp state to flash after it settles

// ---- FreeRTOS ----------------------------------------------------------------------
constexpr size_t RING_SAMPLES = 4096;    // 512 ms of audio between sampler and DSP
constexpr uint32_t SAMPLER_PRIORITY = 20;
constexpr uint32_t DSP_PRIORITY = 5;
constexpr int AUDIO_CORE = 1;

}  // namespace cfg
