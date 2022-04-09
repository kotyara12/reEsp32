#include <time.h>
#include <ctype.h>
#include <string.h>
#include <esp32/rom/rtc.h>
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rLog.h"
#include "reEsp32.h"
#include "reNvs.h"

#if CONFIG_RESTART_DEBUG_INFO && (CONFIG_RESTART_DEBUG_STACK_DEPTH > 0)
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_debug_helpers.h"
#include "soc/soc_memory_layout.h"
#include "soc/cpu.h" 
#endif // CONFIG_RESTART_STACK_DEPTH

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

__NOINIT_ATTR static re_reset_reason_t _reset_reason = RR_UNKNOWN;

shutdown_handler_t _shutdown_handler_app = nullptr;

static void espSystemShutdownHandler()
{
  #if CONFIG_RESTART_DEBUG_INFO
  debugUpdate();
  #endif // CONFIG_RESTART_DEBUG_INFO
}

bool espRegisterShutdownHandler(shutdown_handler_t handler)
{
  esp_err_t err = esp_register_shutdown_handler(handler);
  if (err == ESP_ERR_NO_MEM) {
    rlog_e(logTAG, "Failed to register shutdown handler: no free slots!");
    return false;
  };
  return true;
}

bool espRegisterSystemShutdownHandler() 
{
  return espRegisterShutdownHandler(espSystemShutdownHandler);
}

static void espRestartTimer(void* arg)
{
  esp_restart();
}

void espRestart(re_reset_reason_t reason, uint32_t delay_ms)
{
  rlog_w(logTAG, "******************* Restart system! *******************");
  espSetResetReason(reason);
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

void espSetResetReason(re_reset_reason_t reason)
{
  _reset_reason = reason;
}

re_reset_reason_t espGetResetReason()
{
  re_reset_reason_t ret = _reset_reason;
  _reset_reason = RR_UNKNOWN;
  return ret;
}

const char* getResetReason()
{
  esp_reset_reason_t esp_reason = esp_reset_reason();
  re_reset_reason_t re_reason = espGetResetReason();
  switch (esp_reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   return "POWER ON";
    case ESP_RST_EXT:       return "EXTERNAL PIN";
    case ESP_RST_SW:        
      switch (re_reason) {
        case RR_OTA:        return "OTA UPDATE";
        case RR_OTA_TIMEOUT: return "OTA UPDATE TIMEOUT";
        case RR_COMMAND_RESET: return "COMMAND RESET";
        case RR_HEAP_ALLOCATION_FAILED: return "HEAP ALLOCATION FAILED";
        case RR_WIFI_TIMEOUT: return "WIFI CONNECT TIMEOUT";
        default:            return "SOFTWARE RESET";
      };
    case ESP_RST_PANIC:     
      switch (re_reason) {
        case RR_HEAP_ALLOCATION_FAILED: return "HEAP ALLOCATION FAILED";
        default:            return "EXCEPTION / PANIC";
      };
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
  RESET_REASON esp_reason = rtc_get_reset_reason(cpu_no);
  switch (esp_reason) {
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

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Debug --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

#if CONFIG_RESTART_DEBUG_INFO

__NOINIT_ATTR static re_restart_debug_t _debug_info;

void IRAM_ATTR debugHeapUpdate()
{
  _debug_info.heap_total = heap_caps_get_total_size(MALLOC_CAP_DEFAULT);
  _debug_info.heap_free = heap_caps_get_free_size(MALLOC_CAP_DEFAULT);
  size_t _new_free_min = heap_caps_get_minimum_free_size(MALLOC_CAP_DEFAULT);
  if ((_debug_info.heap_free_min == 0) || (_new_free_min < _debug_info.heap_free_min)) {
    _debug_info.heap_free_min = _new_free_min;
    _debug_info.heap_min_time = time(nullptr);
  };
}

#if CONFIG_RESTART_DEBUG_STACK_DEPTH > 0

void IRAM_ATTR debugBacktraceUpdate()
{
  //Initialize stk_frame with first frame of stack
  esp_backtrace_frame_t stk_frame;
  esp_backtrace_get_start(&(stk_frame.pc), &(stk_frame.sp), &(stk_frame.next_pc)); 
  _debug_info.backtrace[0] = esp_cpu_process_stack_pc(stk_frame.pc);

  bool corrupted = (esp_stack_ptr_is_sane(stk_frame.sp) &&
                    esp_ptr_executable((void*)esp_cpu_process_stack_pc(stk_frame.pc))) ?
                    false : true; 

  #if CONFIG_RESTART_DEBUG_STACK_DEPTH > 1
    uint8_t i = CONFIG_RESTART_DEBUG_STACK_DEPTH;
    while (i-- > 0 && stk_frame.next_pc != 0 && !corrupted) {
      if (!esp_backtrace_get_next_frame(&stk_frame)) {
        corrupted = true;
      };
      _debug_info.backtrace[CONFIG_RESTART_DEBUG_STACK_DEPTH - i] = esp_cpu_process_stack_pc(stk_frame.pc);
    };
  #endif // CONFIG_RESTART_DEBUG_STACK_DEPTH > 1
}

#endif // CONFIG_RESTART_DEBUG_STACK_DEPTH

void IRAM_ATTR debugUpdate()
{
  debugHeapUpdate();
  #if CONFIG_RESTART_DEBUG_STACK_DEPTH > 0
  debugBacktraceUpdate();
  #endif // CONFIG_RESTART_DEBUG_STACK_DEPTH
}

re_restart_debug_t debugGet()
{
  re_restart_debug_t ret;
  memset(&ret, 0, sizeof(re_restart_debug_t));
  esp_reset_reason_t esp_reason = esp_reset_reason();
  if ((esp_reason != ESP_RST_UNKNOWN) && (esp_reason != ESP_RST_POWERON)) {
    ret = _debug_info;
    if (_debug_info.heap_total > heap_caps_get_total_size(MALLOC_CAP_DEFAULT)) {
      memset(&ret, 0, sizeof(re_restart_debug_t));
    };
  };
  memset(&_debug_info, 0, sizeof(re_restart_debug_t));
  return ret;
}

#endif // CONFIG_RESTART_DEBUG_INFO

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Panic --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

// This function will be considered the esp_panic_handler to call in case a panic occurs
void __wrap_esp_panic_handler(void* info) 
{
  #if CONFIG_RESTART_DEBUG_INFO && (CONFIG_RESTART_DEBUG_STACK_DEPTH > 0)
    debugBacktraceUpdate();
  #endif // CONFIG_RESTART_DEBUG_STACK_DEPTH
    
  // Call the original panic handler function to finish processing this error (creating a core dump for example...)
  __real_esp_panic_handler(info);
}

