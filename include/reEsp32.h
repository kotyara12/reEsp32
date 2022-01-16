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
  RR_HEAP_ALLOCATION_ERROR = 3,
  RR_WIFI_TIMEOUT = 4
} re_reset_reason_t;

#ifndef ARDUINO
unsigned long IRAM_ATTR millis();
#endif
int usSleep(uint64_t us);
void timerSet(esp_timer_t *timer, uint32_t timeout);
bool timerTimeout(esp_timer_t *timer);
bool checkTimeout(const unsigned long prevTimestamp, const unsigned long delayValue);
void msTaskDelay(TickType_t value);
void msTaskDelayUntil(TickType_t * const prevTime, TickType_t value);

void* esp_malloc(size_t size);
void* esp_calloc(size_t count, size_t size);

void espRegisterShutdownHandlers();
void espRegisterShutdownHandlerApp(shutdown_handler_t handler_app);
void espRestart(re_reset_reason_t reason);
const char* getResetReason();
const char* getResetReasonRtc(int cpu_no);

#ifdef __cplusplus
}
#endif

#endif // __RE_ESP32_H__

