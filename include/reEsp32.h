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
#include "rLog.h"

// Usage: RE_MEM_CHECK(TAG, item, return NULL);
#define RE_MEM_CHECK(TAG, a, action) if (!(a)) {                               \
  rlog_e(TAG, "%s in \"%s\"::%d", "Memory exhausted", __FUNCTION__, __LINE__); \
  action;                                                                      \
};

// Usage: RE_OK_CHECK(TAG, esp_create_timer(...), return false);
#define RE_OK_CHECK_FIRST(TAG, a, action) esp_err_t err = (a);                                           \
if (err != ESP_OK) {                                                                                     \
  rlog_e(TAG, "\"%s\"::%d failed with code %d (%s)", __FUNCTION__, __LINE__, err, esp_err_to_name(err)); \
  action;                                                                                                \
};
#define RE_OK_CHECK(TAG, a, action) err = (a);                                                           \
if (err != ESP_OK) {                                                                                     \
  rlog_e(TAG, "\"%s\"::%d failed with code %d (%s)", __FUNCTION__, __LINE__, err, esp_err_to_name(err)); \
  action;                                                                                                \
};

typedef struct {
  uint32_t deadline;
} esp_timer_t;

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
  RR_UNKNOWN = 0,
  RR_OTA = 1,
  RR_OTA_TIMEOUT = 2,
  RR_COMMAND_RESET = 3,
  RR_HEAP_ALLOCATION_FAILED = 4,
  RR_WIFI_TIMEOUT = 5
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
void espRestart(re_reset_reason_t reason, uint32_t delay_ms);
const char* getResetReason();
const char* getResetReasonRtc(int cpu_no);

#ifdef __cplusplus
}
#endif

#endif // __RE_ESP32_H__

