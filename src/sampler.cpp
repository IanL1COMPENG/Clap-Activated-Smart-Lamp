#include "sampler.h"

#include <driver/adc.h>

namespace {

hw_timer_t* timer = nullptr;
TaskHandle_t samplerTaskHandle = nullptr;
SampleRing* ringPtr = nullptr;
SamplerStats* statsPtr = nullptr;

// GPIO34 -> ADC1 channel 6
constexpr adc1_channel_t kChannel = ADC1_CHANNEL_6;
static_assert(cfg::PIN_MIC == 34, "update kChannel when changing PIN_MIC");

void IRAM_ATTR onSampleTimer() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(samplerTaskHandle, &woken);
  if (woken) portYIELD_FROM_ISR();
}

void samplerTask(void*) {
  for (;;) {
    // Each timer tick adds one to the notification count. More than one
    // pending means a tick was missed: count it so it shows in the stats.
    const uint32_t ticks = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(10));
    if (ticks == 0) continue;
    if (ticks > 1) statsPtr->missedTicks.fetch_add(ticks - 1, std::memory_order_relaxed);
    const int raw = adc1_get_raw(kChannel);  // 12-bit, 0..4095
    if (!ringPtr->push(static_cast<uint16_t>(raw < 0 ? 0 : raw)))
      statsPtr->ringOverflows.fetch_add(1, std::memory_order_relaxed);
    statsPtr->samples.fetch_add(1, std::memory_order_relaxed);
  }
}

}  // namespace

namespace sampler {

void start(SampleRing* ring, SamplerStats* stats) {
  ringPtr = ring;
  statsPtr = stats;

  adc1_config_width(ADC_WIDTH_BIT_12);
  adc1_config_channel_atten(kChannel, ADC_ATTEN_DB_11);  // ~0-3.1 V input range

  xTaskCreatePinnedToCore(samplerTask, "sampler", 3072, nullptr, cfg::SAMPLER_PRIORITY,
                          &samplerTaskHandle, cfg::AUDIO_CORE);

  // 1 MHz timer tick, alarm every 125 ticks = 8 kHz, auto-reload.
  timer = timerBegin(cfg::SAMPLE_TIMER, cfg::TIMER_PRESCALER, true);
  timerAttachInterrupt(timer, &onSampleTimer, true);
  timerAlarmWrite(timer, 1000000UL / cfg::SAMPLE_RATE_HZ, true);
  timerAlarmEnable(timer);
}

}  // namespace sampler
