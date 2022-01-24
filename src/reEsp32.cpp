#include <time.h>
#include <ctype.h>
#include <esp32/rom/rtc.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rLog.h"
#include "reEsp32.h"
#include "reNvs.h"

/**
 * Timers
 */
#ifndef ARDUINO

#include "esp_attr.h"
#include "esp_timer.h"

static const char* logTAG = "SYSTEM";

unsigned long IRAM_ATTR millis() 
{
  return (unsigned long) (esp_timer_get_time() / 1000ULL);
};

#endif // !defined(ARDUINO)

int usSleep(uint64_t us)
{
  const int us_per_tick = portTICK_PERIOD_MS * 1000;
  if (us < us_per_tick) {
    ets_delay_us((uint32_t) us);
  } else {
    // since vTaskDelay(1) blocks for anywhere between 0 and portTICK_PERIOD_MS, round up to compensate.
    vTaskDelay((us + us_per_tick - 1) / us_per_tick);
  }
  return 0;
}

void timerSet(esp_timer_t *timer, uint32_t timeout) 
{
  timer->deadline = (xTaskGetTickCount() * portTICK_PERIOD_MS) + timeout;
}

bool timerTimeout(esp_timer_t *timer) 
{
  return ((int32_t)timer->deadline - (int32_t)(xTaskGetTickCount() * portTICK_PERIOD_MS)) < 1;
}

bool checkTimeout(const unsigned long prevTimestamp, const unsigned long delayValue)
{
  unsigned long nowTimestamp = millis();
  if (nowTimestamp >= prevTimestamp) {
    return (nowTimestamp >= (prevTimestamp + delayValue));
  }
  else {
    return (nowTimestamp >= delayValue);
  };
}

void msTaskDelay(TickType_t value)
{
  vTaskDelay(value / portTICK_PERIOD_MS);
}

void msTaskDelayUntil(TickType_t * const prevTime, TickType_t value)
{
  vTaskDelayUntil(prevTime, value / portTICK_PERIOD_MS);
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------- Memory allocation --------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if defined(CONFIG_HEAP_ALLOC_FAILED_RETRY) && CONFIG_HEAP_ALLOC_FAILED_RETRY

#define HEAP_ALLOC_ATTEMPTS_MAX 10
#define HEAP_ALLOC_ATTEMPTS_INTERVAL 1000

void* esp_malloc(size_t size)
{
  uint8_t att_count = 1;
  void* addr = nullptr;
  do {
    addr = malloc(size);
    if (addr == nullptr) {
      att_count++;
      vTaskDelay(HEAP_ALLOC_ATTEMPTS_INTERVAL / portTICK_PERIOD_MS);
    };
  } while ((addr == nullptr) && (att_count < HEAP_ALLOC_ATTEMPTS_MAX));
  return addr;
}

void* esp_calloc(size_t count, size_t size)
{
  uint8_t att_count = 0;
  void* addr = nullptr;
  do {
    addr = calloc(count, size);
    if (addr == nullptr) {
      att_count++;
      vTaskDelay(HEAP_ALLOC_ATTEMPTS_INTERVAL / portTICK_PERIOD_MS);
    };
  } while ((addr == nullptr) && (att_count < HEAP_ALLOC_ATTEMPTS_MAX));
  return addr;
}

#else 

void* esp_malloc(size_t size)
{
  return malloc(size);
}

void* esp_calloc(size_t count, size_t size)
{
  return calloc(count, size);
}

#endif // CONFIG_HEAP_ALLOC_FAILED_RETRY

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------- Restarting the device with extended functionality ---------------------------------
// -----------------------------------------------------------------------------------------------------------------------

shutdown_handler_t _shutdown_handler_app = nullptr;

static void espDefaultShutdownHandler()
{
  if (_shutdown_handler_app) {
    _shutdown_handler_app();
  };
}

void espRegisterShutdownHandlers()
{
  esp_err_t err = esp_register_shutdown_handler(espDefaultShutdownHandler);
  if (err != ESP_OK) {
    rlog_e(logTAG, "Failed to register shutdown handler!");
  };
}

void espRegisterShutdownHandlerApp(shutdown_handler_t handler_app)
{
  if (!_shutdown_handler_app) {
    _shutdown_handler_app = handler_app;
  };
};

static void espRestartTimer(void* arg)
{
  esp_restart();
}

void espRestart(re_reset_reason_t reason, uint32_t delay_ms)
{
  rlog_w(logTAG, "******************* Restart system! *******************");
  nvsWrite("system", "reset_reason", OPT_TYPE_U8, &reason);
  if (delay_ms > 0) {
    esp_timer_create_args_t tmrCfg;
    tmrCfg.callback = espRestartTimer;
    tmrCfg.name = "timer_restart";
    esp_timer_handle_t tmrHandle;
    if ((esp_timer_create(&tmrCfg, &tmrHandle) == ESP_OK) 
     && (esp_timer_start_once(tmrHandle, delay_ms * 1000) == ESP_OK)) {
      return;
    };
  };
  esp_restart();
}

re_reset_reason_t espGetResetReason()
{
  re_reset_reason_t ret = RR_UNKNOWN;
  nvsRead("system", "reset_reason", OPT_TYPE_U8, &ret);
  if (ret != RR_UNKNOWN) {
    re_reset_reason_t clr = RR_UNKNOWN;
    nvsWrite("system", "reset_reason", OPT_TYPE_U8, &clr);
  };
  return ret;
}

const char* getResetReason()
{
  esp_reset_reason_t reason = esp_reset_reason();
  switch (reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWER ON";
    case ESP_RST_EXT:       return "EXTERNAL PIN";
    case ESP_RST_SW:        
      switch (espGetResetReason()) {
        case RR_OTA:        return "OTA UPDATE";
        case RR_OTA_TIMEOUT: return "OTA UPDATE TIMEOUT";
        case RR_COMMAND_RESET: return "COMMAND RESET";
        case RR_HEAP_ALLOCATION_FAILED: return "HEAP ALLOCATION FAILED";
        case RR_WIFI_TIMEOUT: return "WIFI CONNECT TIMEOUT";
        default:            return "SOFTWARE RESET";
      };
    case ESP_RST_PANIC:     return "EXCEPTION / PANIC";
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";
    case ESP_RST_WDT:       return "WATCHDOGS";
    case ESP_RST_DEEPSLEEP: return "EXITING DEEP SLLEP MODE";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default :               return "NO MEAN";
  };
} 

const char* getResetReasonRtc(int cpu_no)
{
  RESET_REASON reason = rtc_get_reset_reason(cpu_no);
  switch (reason) {
    case POWERON_RESET:           return "Vbat power on reset";
    case SW_RESET:                return "Software reset digital core";
    case OWDT_RESET:              return "Legacy watch dog reset digital core";
    case DEEPSLEEP_RESET:         return "Deep Sleep reset digital core";
    case SDIO_RESET:              return "Reset by SLC module, reset digital core";
    case TG0WDT_SYS_RESET:        return "Timer Group0 Watch dog reset digital core";
    case TG1WDT_SYS_RESET:        return "Timer Group1 Watch dog reset digital core";
    case RTCWDT_SYS_RESET:        return "RTC Watch dog Reset digital core";
    case INTRUSION_RESET:         return "Instrusion tested to reset CPU";
    case TGWDT_CPU_RESET:         return "Time Group reset CPU";
    case SW_CPU_RESET:            return "Software reset CPU";
    case RTCWDT_CPU_RESET:        return "RTC Watch dog Reset CPU";
    case EXT_CPU_RESET:           return "For APP CPU, reseted by PRO CPU";
    case RTCWDT_BROWN_OUT_RESET:  return "Reset when the vdd voltage is not stable";
    case RTCWDT_RTC_RESET:        return "RTC Watch dog reset digital core and rtc module";
    default :                     return "NO MEAN";
  };
} 

