#include <time.h>
#include <ctype.h>
#include <string.h>
#include <esp32/rom/rtc.h>
#include "esp_system.h"
#include "esp_heap_caps.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "rLog.h"
#include "reEsp32.h"
#include "reNvs.h"

#include "esp_log.h"
#if CONFIG_RESTART_DEBUG_INFO && (CONFIG_RESTART_DEBUG_STACK_DEPTH > 0)
#include "esp_types.h"
#include "esp_attr.h"
#include "esp_err.h"
#include "esp_debug_helpers.h"
#include "soc/soc_memory_layout.h"
#if ESP_IDF_VERSION_MAJOR < 5
  #include "soc/cpu.h"
#else
  #include "esp_cpu_utils.h"
#endif // ESP_IDF_VERSION_MAJOR
#endif // CONFIG_RESTART_DEBUG_STACK_DEPTH

/**
 * Timers
 */
#ifndef ARDUINO

#include "esp_attr.h"
#include "esp_timer.h"

#if CONFIG_RLOG_PROJECT_LEVEL > RLOG_LEVEL_NONE
static const char* logTAG = "SYSTEM";
#endif // CONFIG_RLOG_PROJECT_LEVEL

unsigned long millis() 
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

float esp_heap_free_percent()
{
  return 100.0 * ((float)heap_caps_get_free_size(MALLOC_CAP_DEFAULT)/(float)heap_caps_get_total_size(MALLOC_CAP_DEFAULT));
}

float esp_heap_free_check()
{
  #if defined(CONFIG_HEAP_MINIMAL_SIZE_PERCENT) && (CONFIG_HEAP_MINIMAL_SIZE_PERCENT > 0) && (CONFIG_HEAP_MINIMAL_SIZE_PERCENT < 100)
    return esp_heap_free_percent() > CONFIG_HEAP_MINIMAL_SIZE_PERCENT;
  #else 
    return true;
  #endif // CONFIG_HEAP_MINIMAL_SIZE_PERCENT
}

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
// ------------------------------------ Device restart timer when something went wrong -----------------------------------
// -----------------------------------------------------------------------------------------------------------------------

static void espRestartTimerEnd(void* arg)
{
  if (arg == nullptr) {
    espRestart(RR_UNKNOWN);
  } else {
    re_restart_timer_t* restart_timer = (re_restart_timer_t*)arg;
    rlog_w(logTAG, "Device restart timer [ reason # %d ] timeout...", restart_timer->reason);
    espRestart(restart_timer->reason);
  };
}

bool espRestartTimerInit(re_restart_timer_t* restart_timer, re_reset_reason_t reason, const char* tmr_name)
{
  if (restart_timer == nullptr) {
    rlog_e(logTAG, "Failed to initialize device restart timer: argument is NULL");
    return false;
  };
  
  if (restart_timer->timer == nullptr) {
    esp_timer_create_args_t tmrCfg;
    memset(&tmrCfg, 0, sizeof(tmrCfg));
    tmrCfg.callback = espRestartTimerEnd;
    if (tmr_name) {
      tmrCfg.name = tmr_name;
    } else {
      tmrCfg.name = "wdt_restart";
    };
    tmrCfg.arg = restart_timer;
    esp_err_t err = esp_timer_create(&tmrCfg, &restart_timer->timer);
    if (err != ESP_OK) {
      rlog_e(logTAG, "Failed to create device restart timer: %d %s", err, esp_err_to_name(err));
      return false;
    };
  };
  restart_timer->reason = reason;
  return true;
}

void espRestartTimerStart(re_restart_timer_t* restart_timer, re_reset_reason_t reason, uint64_t delay_ms, bool override)
{
  if (restart_timer == nullptr) {
    rlog_e(logTAG, "Failed to start device restart timer: argument is NULL");
  };

  if (delay_ms > 0) {
    restart_timer->reason = reason;

    if ((restart_timer->timer != nullptr) && esp_timer_is_active(restart_timer->timer)) {
      if (override) {
        espRestartTimerBreak(restart_timer);
      } else {
        return;
      };
    };
      
    if ((restart_timer->timer != nullptr) || espRestartTimerInit(restart_timer, reason, nullptr)) {
      esp_err_t err = esp_timer_start_once(restart_timer->timer, delay_ms * 1000);
      if (err == ESP_OK) {
        rlog_w(logTAG, "Device restart timer started for %d s due to reason # %d", (uint32_t)(delay_ms / 1000), reason);
        return;
      } else {
        rlog_e(logTAG, "Failed to start device restart timer: %d %s", err, esp_err_to_name(err));
      };
    };
  } else {
    espRestart(reason);
  };
}

void espRestartTimerBreak(re_restart_timer_t* restart_timer)
{
  if (restart_timer == nullptr) {
    rlog_e(logTAG, "Failed to stop device restart timer: argument is NULL");
  };

  if (restart_timer->timer != nullptr) {
    if (esp_timer_is_active(restart_timer->timer)) {
      esp_err_t err = esp_timer_stop(restart_timer->timer);
      if (err == ESP_OK) {
        rlog_i(logTAG, "Device restart timer for reason # %d is stopped", restart_timer->reason);
      } else {
        rlog_e(logTAG, "Failed to stop device restart timer: %d %s", err, esp_err_to_name(err));
      };
    };
  };
}

void espRestartTimerFree(re_restart_timer_t* restart_timer)
{
  if (restart_timer == nullptr) {
    rlog_e(logTAG, "Failed to free device restart timer: argument is NULL");
  };

  if (restart_timer->timer != nullptr) {
    esp_err_t err = ESP_OK;
    if (esp_timer_is_active(restart_timer->timer)) {
      err = esp_timer_stop(restart_timer->timer);
      if (err == ESP_OK) {
        rlog_i(logTAG, "Device restart timer for reason # %d is stopped", restart_timer->reason);
      } else {
        rlog_e(logTAG, "Failed to stop device restart timer: %d %s", err, esp_err_to_name(err));
      };
    };
    if (err == ESP_OK) {
      err = esp_timer_delete(restart_timer->timer);
      if (err == ESP_OK) {
        restart_timer->timer = nullptr;
        rlog_i(logTAG, "Device restart timer for reason # %d is deleted", restart_timer->reason);
      } else {
        rlog_e(logTAG, "Failed to delete device restart timer: %d %s", err, esp_err_to_name(err));
      };
    };
  };
}

// -----------------------------------------------------------------------------------------------------------------------
// ----------------------------------- Restarting the device with extended functionality ---------------------------------
// -----------------------------------------------------------------------------------------------------------------------

shutdown_handler_t _shutdown_handler_app = nullptr;

static void IRAM_ATTR espSystemShutdownHandler()
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

static __NOINIT_ATTR re_reset_reason_t _reset_reason_ext = RR_UNKNOWN;

void IRAM_ATTR espSetResetReason(re_reset_reason_t reason)
{
  _reset_reason_ext = reason;
}

re_reset_reason_t espGetResetReason()
{
  return _reset_reason_ext;
}

const char* getResetReason()
{
  esp_reset_reason_t esp_reason = esp_reset_reason();
  switch (esp_reason) {
    case ESP_RST_UNKNOWN:   return "UNKNOWN";
    case ESP_RST_POWERON:   
      switch (_reset_reason_ext) {
        case RR_BAT_LOW:    return "BATTERY LOW";
        default:            return "POWER ON";
      };
    case ESP_RST_EXT:       return "EXTERNAL PIN";
    case ESP_RST_SW:        
      switch (_reset_reason_ext) {
        case RR_ERROR:        return "FIRMWARE ERROR";
        case RR_OTA:        return "OTA UPDATE OK";
        case RR_OTA_TIMEOUT: return "OTA UPDATE TIMEOUT";
        case RR_OTA_FAILED: return "OTA FAILED (ROLLBACK)";
        case RR_COMMAND_RESET: return "COMMAND RESET";
        case RR_HEAP_ALLOCATION_FAILED: return "HEAP ALLOCATION FAILED";
        case RR_WIFI_TIMEOUT: return "WIFI CONNECT TIMEOUT";
        case RR_MQTT_TIMEOUT: return "MQTT CONNECT TIMEOUT";
        default:            return "SOFTWARE RESET";
      };
    case ESP_RST_PANIC:     
      switch (_reset_reason_ext) {
        case RR_HEAP_ALLOCATION_FAILED: return "HEAP ALLOCATION FAILED";
        default:            return "EXCEPTION / PANIC";
      };
    case ESP_RST_INT_WDT:   return "INTERRUPT WATCHDOG";
    case ESP_RST_TASK_WDT:  return "TASK WATCHDOG";
    case ESP_RST_WDT:
      switch (_reset_reason_ext) {
        case RR_ERROR:        return "FIRMWARE ERROR (WD)";
        case RR_OTA:        return "OTA UPDATE OK (WD)";
        case RR_OTA_TIMEOUT: return "OTA UPDATE TIMEOUT (WD)";
        case RR_OTA_FAILED: return "OTA FAILED (ROLLBACK) (WD)";
        case RR_COMMAND_RESET: return "COMMAND RESET (WD)";
        case RR_HEAP_ALLOCATION_FAILED: return "HEAP ALLOCATION FAILED (WD)";
        case RR_WIFI_TIMEOUT: return "WIFI CONNECT TIMEOUT (WD)";
        case RR_MQTT_TIMEOUT: return "MQTT CONNECT TIMEOUT (WD)";
        default:            return "WATCHDOGS";
      };
    case ESP_RST_DEEPSLEEP: return "EXITING DEEP SLLEP MODE";
    case ESP_RST_BROWNOUT:  return "BROWNOUT";
    case ESP_RST_SDIO:      return "SDIO";
    default :               return "NO MEAN";
  };
  _reset_reason_ext = RR_UNKNOWN;
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

void espRestart(re_reset_reason_t reason)
{
  rlog_w(logTAG, "******************* Restart system! *******************");
  espSetResetReason(reason);
  esp_restart();
}

// -----------------------------------------------------------------------------------------------------------------------
// -------------------------------------------------------- Debug --------------------------------------------------------
// -----------------------------------------------------------------------------------------------------------------------

void disbleEspIdfLogs()
{
  esp_log_level_set("wifi", ESP_LOG_ERROR);
  esp_log_level_set("event", ESP_LOG_ERROR);
  esp_log_level_set("tcpip", ESP_LOG_ERROR);
  esp_log_level_set("tcpip_adapter", ESP_LOG_ERROR);
  esp_log_level_set("phy", ESP_LOG_ERROR);
  esp_log_level_set("phy_version", ESP_LOG_ERROR);
  esp_log_level_set("phy: phy_version", ESP_LOG_ERROR);
  esp_log_level_set("i2c", ESP_LOG_ERROR);
  esp_log_level_set("esp_https_ota", ESP_LOG_INFO);
  esp_log_level_set("HTTP_CLIENT", ESP_LOG_NONE);
  esp_log_level_set("MQTT_CLIENT", ESP_LOG_NONE);
  esp_log_level_set("TRANS_TCP", ESP_LOG_NONE);
}

#if CONFIG_RESTART_DEBUG_INFO

static __NOINIT_ATTR re_restart_debug_t _debug_info;

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
  _reset_reason_ext = RR_UNKNOWN;

  #if CONFIG_RESTART_DEBUG_INFO && (CONFIG_RESTART_DEBUG_STACK_DEPTH > 0)
    debugBacktraceUpdate();
  #endif // CONFIG_RESTART_DEBUG_STACK_DEPTH
    
  // Call the original panic handler function to finish processing this error (creating a core dump for example...)
  __real_esp_panic_handler(info);
}

