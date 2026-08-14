#include "app_hal_bridge.h"
#include "adc_sampler.h"
#include "app_context.h"
#include "comm_service.h"
#include "dc_motor_ol.h"
#include "foc_link.h"
#include "lvgl.h"
#include "mecanum.h"
#include "mecanum_odometry.h"
#include "mjpeg_scheduler.h"
#include "runtime_monitor.h"
#include "safety_manager.h"
#include "ui_settings.h"

static uint8_t s_uart5_rx_byte;
static volatile uint32_t s_tim7_frame_tick;

static void app_hal_bridge_arm_uart5_rx(void)
{
    const AppContext *context = AppContext_Get();

    if (context->uart5 != NULL)
    {
        (void)HAL_UART_Receive_IT(context->uart5, &s_uart5_rx_byte, 1U);
    }
}

void AppHalBridge_Init(void)
{
    s_uart5_rx_byte = 0U;
    s_tim7_frame_tick = 0U;
    app_hal_bridge_arm_uart5_rx();
}

uint32_t AppHalBridge_GetTim7FrameTick(void)
{
    return s_tim7_frame_tick;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return;
    }

    if (uart->Instance == UART5)
    {
        CommService_UartRxByteFromISR(s_uart5_rx_byte);
        app_hal_bridge_arm_uart5_rx();
    }
    else if (uart->Instance == UART4)
    {
        FOC_Link_UartRxCompleteFromISR(uart);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *uart)
{
    CommService_UartTxCompleteFromISR(uart);
    FOC_Link_UartTxCompleteFromISR(uart);
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *uart)
{
    if (uart == NULL)
    {
        return;
    }

    if (uart->Instance == UART5)
    {
        app_hal_bridge_arm_uart5_rx();
    }
    else if (uart->Instance == UART4)
    {
        FOC_Link_UartErrorFromISR(uart);
    }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *timer)
{
    if (timer == NULL)
    {
        return;
    }

    if (timer->Instance == TIM6)
    {
        lv_tick_inc(1U);
        UI_Settings_BuzzerTick1ms();
    }
    else if (timer->Instance == TIM7)
    {
        MJPEG_Scheduler_OnTim7Tick();
        s_tim7_frame_tick++;
    }
    else if (timer->Instance == TIM13)
    {
        uint32_t control_start = Safety_ControlTimingStart();

        RuntimeMonitor_ControlTickFromISR();
        Safety_ControlTick10ms();
        FOC_Link_SetSafetyInhibit(
            (Safety_IsMotionAllowed() == 0U) ? 1U : 0U);
        if (Safety_IsMotionAllowed() != 0U)
        {
            Mecanum_Tick10ms();
            if (Safety_IsMotionAllowed() != 0U)
            {
                DCMotor_OL_ApplyPendingCommands();
            }
            else
            {
                Mecanum_EmergencyStop();
            }
        }
        else
        {
            Mecanum_EmergencyStop();
        }

        DCMotor_OL_Tick10ms();
        MecanumOdometry_Update10ms();
        Safety_SetMotionActive(
            ((Mecanum_IsMotionActive() != 0U) ||
             (DCMotor_OL_IsMotionActive() != 0U)) ? 1U : 0U);

        Safety_ControlTimingFinish(control_start);
        if (Safety_IsMotionAllowed() == 0U)
        {
            FOC_Link_SetSafetyInhibit(1U);
            Mecanum_EmergencyStop();
        }
    }
    else if (timer->Instance == TIM16)
    {
        AdcSampler_RequestFromISR();
    }
}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *adc)
{
    AdcSampler_ConversionCompleteFromISR(adc);
}
