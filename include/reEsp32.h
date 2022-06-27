/* 
   EN: System utilities and features to ESP32 (ESP-IDF)
   RU: Системные утилиты и функции для ESP32 (ESP-IDF)
   --------------------------
   (с) 2021 Разживин Александр | Razzhivin Alexander
   kotyara12@yandex.ru | https://kotyara12.ru | tg: @kotyara1971
*/

#ifndef __RE_ESP32_H__
#define __RE_ESP32_H__

#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "rLog.h"

// Usage: RE_MEM_CHECK(TAG, item, return NULL);
#define RE_MEM_CHECK(a, action) if ((a) == nullptr) { \
  rlog_e(logTAG, "%s in \"%s\"::%d", "Memory exhausted", __FUNCTION__, __LINE__); \
  action; \
};

#define RE_MEM_CHECK_EVENT(a, action) if ((a) == nullptr) { \
  eventLoopPostError(RE_SYS_ERROR, ESP_ERR_NO_MEM); \
  rlog_e(logTAG, "%s in \"%s\"::%d", "Memory exhausted", __FUNCTION__, __LINE__); \
  action; \
};

// Usage: RE_OK_CHECK(esp_create_timer(...), return false);
#define RE_OK_CHECK(a, action) do { \
  esp_err_t __err = (a); \
  if (__err != ESP_OK) { \
    rlog_e(logTAG, "\"%s\"::%d failed with code %d (%s)", __FUNCTION__, __LINE__, __err, esp_err_to_name(__err)); \
    action; \
  }; \
} while (0);

#define RE_OK_CHECK_EVENT(a, action) do { \
  esp_err_t __err = (a); \
  if (__err != ESP_OK) { \
    eventLoopPostError(RE_SYS_ERROR, __err); \
    rlog_e(logTAG, "\"%s\"::%d failed with code %d (%s)", __FUNCTION__, __LINE__, __err, esp_err_to_name(__err)); \
    action; \
  }; \
} while (0);

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
  RR_OTA_FAILED = 3,
  RR_COMMAND_RESET = 4,
  RR_HEAP_ALLOCATION_FAILED = 5,
  RR_WIFI_TIMEOUT = 6,
  RR_BAT_LOW = 7
} re_reset_reason_t;

typedef struct {
  re_reset_reason_t reason;
  esp_timer_handle_t timer;
} re_restart_timer_t;

#if CONFIG_RESTART_DEBUG_INFO

typedef struct {
  size_t heap_total;
  size_t heap_free;
  size_t heap_free_min;
  time_t heap_min_time;
  #if CONFIG_RESTART_DEBUG_STACK_DEPTH > 0
  uint32_t backtrace[CONFIG_RESTART_DEBUG_STACK_DEPTH];
  #endif // CONFIG_RESTART_DEBUG_STACK_DEPTH
} re_restart_debug_t;

#endif // CONFIG_RESTART_DEBUG_INFO

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
float esp_heap_free_percent();
float esp_heap_free_check();

bool espRegisterShutdownHandler(shutdown_handler_t handler);
bool espRegisterSystemShutdownHandler();

void espSetResetReason(re_reset_reason_t reason);
re_reset_reason_t espGetResetReason();
const char* getResetReason();
const char* getResetReasonRtc(int cpu_no);
void espRestart(re_reset_reason_t reason);

bool espRestartTimerInit(re_restart_timer_t* restart_timer, re_reset_reason_t reason);
void espRestartTimerStart(re_restart_timer_t* restart_timer, re_reset_reason_t reason, uint64_t delay_ms, bool override);
void espRestartTimerBreak(re_restart_timer_t* restart_timer);
void espRestartTimerFree(re_restart_timer_t* restart_timer);
#define espRestartTimerStartS(restart_timer, reason, delay_s, override) espRestartTimerStart(restart_timer, reason, delay_s*1000, override)
#define espRestartTimerStartM(restart_timer, reason, delay_m, override) espRestartTimerStart(restart_timer, reason, delay_m*1000*60, override)
#define espRestartTimerStartH(restart_timer, reason, delay_h, override) espRestartTimerStart(restart_timer, reason, delay_h*1000*60*60, override)

void disbleEspIdfLogs();
#if CONFIG_RESTART_DEBUG_INFO
void debugHeapUpdate();
void debugUpdate();
re_restart_debug_t debugGet();
#endif // CONFIG_RESTART_DEBUG_INFO

void __real_esp_panic_handler(void*);
void __wrap_esp_panic_handler(void* info);

#ifdef __cplusplus
}
#endif

#endif // __RE_ESP32_H__

