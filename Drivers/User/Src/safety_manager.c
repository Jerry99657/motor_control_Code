#include "safety_manager.h"

#define SAFETY_CONTROL_WARN_US             1000U
#define SAFETY_CONTROL_HARD_LIMIT_US       8000U

static volatile uint32_t s_fault_flags = SAFETY_FAULT_NONE;
static volatile uint32_t s_warning_flags = SAFETY_WARNING_NONE;
static volatile uint8_t s_motion_active = 0U;
static volatile uint8_t s_external_motion_active = 0U;
static volatile uint8_t s_estop_input_active = 0U;
static volatile uint32_t s_control_max_cycles = 0U;
static volatile uint32_t s_control_overrun_count = 0U;
static volatile uint32_t s_output_blocked_count = 0U;
static uint32_t s_cycles_per_us = 1U;

static uint32_t safety_lock(void)
{
    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    __DMB();
    return primask;
}

static void safety_unlock(uint32_t primask)
{
    __DMB();
    if (primask == 0U)
    {
        __enable_irq();
    }
}

void Safety_Init(void)
{
    uint32_t primask = safety_lock();

    s_fault_flags = SAFETY_FAULT_NONE;
    s_warning_flags = SAFETY_WARNING_NONE;
    s_motion_active = 0U;
    s_external_motion_active = 0U;
    s_estop_input_active = 0U;
    s_control_max_cycles = 0U;
    s_control_overrun_count = 0U;
    s_output_blocked_count = 0U;

    safety_unlock(primask);

    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
    DWT->CYCCNT = 0U;
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;

    s_cycles_per_us = SystemCoreClock / 1000000U;
    if (s_cycles_per_us == 0U)
    {
        s_cycles_per_us = 1U;
    }
}

void Safety_ControlTick10ms(void)
{
    s_estop_input_active =
        (HAL_GPIO_ReadPin(Key2_GPIO_Port, Key2_Pin) == GPIO_PIN_RESET) ? 1U : 0U;

    if ((s_estop_input_active != 0U) &&
        ((s_motion_active != 0U) || (s_external_motion_active != 0U)))
    {
        s_fault_flags |= SAFETY_FAULT_ESTOP;
    }
    else if (s_estop_input_active == 0U)
    {
        /* KEY2 is a momentary safety stop. Other safety faults stay latched. */
        s_fault_flags &= ~((uint32_t)SAFETY_FAULT_ESTOP);
    }

}

void Safety_SetMotionActive(uint8_t motion_active)
{
    s_motion_active = (motion_active != 0U) ? 1U : 0U;
}

void Safety_SetExternalMotionActive(uint8_t motion_active)
{
    s_external_motion_active = (motion_active != 0U) ? 1U : 0U;
}

void Safety_LatchFault(uint32_t fault_mask)
{
    uint32_t primask;

    if (fault_mask == SAFETY_FAULT_NONE)
    {
        return;
    }

    primask = safety_lock();
    s_fault_flags |= fault_mask;
    safety_unlock(primask);
}

void Safety_SetWarning(uint32_t warning_mask, uint8_t active)
{
    uint32_t primask;

    if (warning_mask == SAFETY_WARNING_NONE)
    {
        return;
    }

    primask = safety_lock();
    if (active != 0U)
    {
        s_warning_flags |= warning_mask;
    }
    else
    {
        s_warning_flags &= ~warning_mask;
    }
    safety_unlock(primask);
}

uint8_t Safety_IsMotionAllowed(void)
{
    return ((s_fault_flags == SAFETY_FAULT_NONE) && (s_estop_input_active == 0U)) ? 1U : 0U;
}

uint32_t Safety_GetFaults(void)
{
    return s_fault_flags;
}

uint32_t Safety_GetWarnings(void)
{
    return s_warning_flags;
}

void Safety_RecordOutputBlocked(void)
{
    if (s_output_blocked_count != UINT32_MAX)
    {
        s_output_blocked_count++;
    }
}

uint32_t Safety_GetOutputBlockedCount(void)
{
    return s_output_blocked_count;
}

uint32_t Safety_ControlTimingStart(void)
{
    return DWT->CYCCNT;
}

void Safety_ControlTimingFinish(uint32_t start_cycles)
{
    uint32_t elapsed = DWT->CYCCNT - start_cycles;
    uint32_t warn_cycles = s_cycles_per_us * SAFETY_CONTROL_WARN_US;
    uint32_t hard_cycles = s_cycles_per_us * SAFETY_CONTROL_HARD_LIMIT_US;

    if (elapsed > s_control_max_cycles)
    {
        s_control_max_cycles = elapsed;
    }

    if (elapsed > warn_cycles)
    {
        s_control_overrun_count++;
    }

    if (elapsed > hard_cycles)
    {
        s_fault_flags |= SAFETY_FAULT_CONTROL_OVERRUN;
    }
}

uint32_t Safety_GetControlMaxCycles(void)
{
    return s_control_max_cycles;
}

uint32_t Safety_GetControlOverrunCount(void)
{
    return s_control_overrun_count;
}
