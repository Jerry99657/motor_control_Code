#include "app_runtime.h"
#include "adc_sampler.h"
#include "app_boot.h"
#include "app_event.h"
#include "app_hal_bridge.h"
#include "app_scheduler.h"
#include "battery_monitor.h"
#include "camera_service.h"
#include "command_protocol.h"
#include "command_control.h"
#include "comm_service.h"
#include "foc_link.h"
#include "imu_service.h"
#include "lvgl_app.h"
#include "runtime_monitor.h"
#include "telemetry_service.h"
#include "ui_settings.h"

static AppScheduler s_scheduler;
static uint8_t s_initialized;

static void app_runtime_comm_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    AppHalBridge_Process();
    CommService_Process();
}

static void app_runtime_foc_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    FOC_Link_Process();
}

static void app_runtime_event_task(uint32_t now, void *context)
{
    AppEvent event;

    (void)now;
    (void)context;
    while (AppEvent_Get(&event) != 0U)
    {
        if (event.type == APP_EVENT_COMM_RX)
        {
            CommandProtocol_Receive(event.channel, event.payload,
                                    event.length);
        }
        AppEvent_MarkHandled();
    }
}

static void app_runtime_imu_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    IMU_Service_Process();
}

static void app_runtime_camera_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    Camera_Service_Process();
}

static void app_runtime_adc_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    AdcSampler_Process();
}

static void app_runtime_battery_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    BatteryMonitor_Process();
    UI_Settings_SetLowBatteryAlert(BatteryMonitor_IsLow());
}

static void app_runtime_boot_pre_ui_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    AppBoot_ProcessPreUi();
}

static void app_runtime_ui_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    LVGL_App_Process();
}

static void app_runtime_telemetry_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    TelemetryService_Process();
}

static void app_runtime_boot_post_ui_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    AppBoot_ProcessPostUi();
}

static void app_runtime_heartbeat_task(uint32_t now, void *context)
{
    (void)now;
    (void)context;
    RuntimeMonitor_MainLoopHeartbeat();
}

static void app_runtime_add_task(AppSchedulerTaskFn function)
{
    if (AppScheduler_Add(&s_scheduler, function, NULL, 0U, 0U) == 0U)
    {
        Error_Handler();
    }
}

void AppRuntime_EarlyInit(void)
{
    RuntimeMonitor_EarlyInit();
}

void AppRuntime_Init(const AppContext *context)
{
    AppContext_Init(context);
    AppEvent_Init();
    CommandControl_Init();
    CommandProtocol_Init();
    TelemetryService_Init();
    AppBoot_Run();

    AppScheduler_Init(&s_scheduler);
    app_runtime_add_task(app_runtime_comm_task);
    app_runtime_add_task(app_runtime_event_task);
    app_runtime_add_task(app_runtime_foc_task);
    app_runtime_add_task(app_runtime_imu_task);
    app_runtime_add_task(app_runtime_camera_task);
    app_runtime_add_task(app_runtime_adc_task);
    app_runtime_add_task(app_runtime_battery_task);
    app_runtime_add_task(app_runtime_boot_pre_ui_task);
    app_runtime_add_task(app_runtime_ui_task);
    app_runtime_add_task(app_runtime_telemetry_task);
    app_runtime_add_task(app_runtime_boot_post_ui_task);
    app_runtime_add_task(app_runtime_heartbeat_task);
    s_initialized = 1U;
}

// #define APP_IWDG_TEST_ENABLE 1U
void AppRuntime_Process(void)
{
//测试看门狗临时加入 短按Key1进入死循环，触发看门狗复位
// #if APP_IWDG_TEST_ENABLE
//     if (HAL_GPIO_ReadPin(Key1_GPIO_Port, Key1_Pin) == GPIO_PIN_RESET)
//     {
//         /* 先关闭电机PWM、发送FOC停止命令，再关闭中断停止喂狗 */
//         RuntimeMonitor_SafeShutdownImmediate();

//         while (1)
//         {
//             __NOP();
//         }
//     }
// #endif
//测试看门狗临时加入
    if (s_initialized != 0U)
    {
        AppScheduler_Run(&s_scheduler, HAL_GetTick());
    }
}

void AppRuntime_GetSchedulerStats(AppSchedulerStats *stats)
{
    AppScheduler_GetStats(&s_scheduler, stats);
}
