#pragma once

#include <Arduino.h>
#include <esp_system.h>
#include "config.h"

// RAM-only evidence. No flash writes and no additional serial output path.
namespace diagnostics {
struct State {
  uint32_t bootId = 0;
  esp_reset_reason_t resetReason = ESP_RST_UNKNOWN;
  uint32_t maxControlMs = 0;
  uint32_t stalls = 0;
};
inline State& st() { static State s; return s; }

inline void begin() {
  st().bootId = esp_random();
  st().resetReason = esp_reset_reason();
}

inline const char* resetName() {
  switch (st().resetReason) {
    case ESP_RST_POWERON: return "POWERON";
    case ESP_RST_EXT: return "EXT";
    case ESP_RST_SW: return "SW";
    case ESP_RST_PANIC: return "PANIC";
    case ESP_RST_INT_WDT: return "INT_WDT";
    case ESP_RST_TASK_WDT: return "TASK_WDT";
    case ESP_RST_WDT: return "WDT";
    case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
    case ESP_RST_BROWNOUT: return "BROWNOUT";
    case ESP_RST_SDIO: return "SDIO";
    default: return "UNKNOWN";
  }
}

inline void noteControlGap(uint32_t elapsed) {
  if (elapsed > st().maxControlMs) st().maxControlMs = elapsed;
  if (elapsed > CONTROL_STALL_MS) ++st().stalls;
}
}  // namespace diagnostics
