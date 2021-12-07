/* 
   EN: System utilities and features to ESP32 (ESP-IDF)
   RU: Системные утилиты и функции для ESP32 (ESP-IDF)
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_ESP32_H__
#define __RE_ESP32_H__

#include "freertos/FreeRTOS.h"

typedef struct {
  uint32_t deadline;
} esp_timer_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  RR_UNKNOWN = 0,
  RR_OTA = 1,
  RR_COMMAND_RESET = 2,
  RR_HEAP_ALLOCATION_ERROR = 3
} re_reset_reason_t;

typedef void (*reset_callback_t) (re_reset_reason_t reason);

#ifndef ARDUINO
unsigned long IRAM_ATTR millis();
#endif
int usSleep(uint64_t us);
void timerSet(esp_timer_t *timer, uint32_t timeout);
bool timerTimeout(esp_timer_t *timer);
bool checkTimeout(const unsigned long prevTimestamp, const unsigned long delayValue);
void msTaskDelay(TickType_t value);
void msTaskDelayUntil(TickType_t * const prevTime, TickType_t value);

void espRestartSetCallback(reset_callback_t reset_callback);
void espRestart(re_reset_reason_t reason);
const char* getResetReason();
const char* getResetReasonRtc(int cpu_no);

#ifdef __cplusplus
}
#endif

#endif // __RE_ESP32_H__

