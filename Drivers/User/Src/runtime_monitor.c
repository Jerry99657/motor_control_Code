#include "runtime_monitor.h"

#include <stddef.h>
#include <string.h>

extern uint8_t _estack;
extern uint32_t _Min_Stack_Size;

#define RUNTIME_MONITOR_FAULT_MAGIC             0x524D4631UL /* "RMF1" */
#define RUNTIME_MONITOR_FAULT_VERSION           1U
#define RUNTIME_MONITOR_BACKUP_ADDRESS          D3_BKPSRAM_BASE
#define RUNTIME_MONITOR_FEED_PERIOD_MS          250U
#define RUNTIME_MONITOR_STACK_CHECK_PERIOD_MS   1000U
#define RUNTIME_MONITOR_CONTROL_STALL_MS         1000U
#define RUNTIME_MONITOR_STACK_PATTERN           0xA5A5A5A5UL
#define RUNTIME_MONITOR_STACK_GUARD_WORDS        8U

typedef struct
{
    uint32_t magic;
    uint16_t version;
    uint16_t size;
    uint32_t crc32;
    uint32_t fault_type;
    uint32_t reset_flags;
    uint32_t tick_ms;
    uint32_t exception_lr;
    uint32_t stacked_r0;
    uint32_t stacked_r1;
    uint32_t stacked_r2;
    uint32_t stacked_r3;
    uint32_t stacked_r12;
    uint32_t stacked_lr;
    uint32_t stacked_pc;
    uint32_t stacked_psr;
    uint32_t cfsr;
    uint32_t hfsr;
    uint32_t dfsr;
    uint32_t afsr;
    uint32_t bfar;
    uint32_t mmfar;
    uint32_t msp;
    uint32_t psp;
    uint32_t control_sequence;
    uint32_t foreground_sequence;
    uint32_t reserved[7];
} RuntimeMonitorFaultRecord;

_Static_assert((sizeof(RuntimeMonitorFaultRecord) & 31U) == 0U,
               "Backup fault record must be cache-line aligned");

static IWDG_HandleTypeDef *s_watchdog = NULL;
static RuntimeMonitorState s_state = RUNTIME_MONITOR_STATE_EARLY;
static RuntimeMonitorFaultRecord s_last_fault;
static RuntimeMonitorFaultRecord s_fault_work;
static uint32_t s_reset_flags = 0U;
static volatile uint32_t s_foreground_sequence = 0U;
static volatile uint32_t s_control_sequence = 0U;
static volatile uint32_t s_last_control_tick = 0U;
static uint32_t s_fed_foreground_sequence = 0U;
static uint32_t s_fed_control_sequence = 0U;
static uint32_t s_last_foreground_tick = 0U;
static uint32_t s_last_feed_tick = 0U;
static uint32_t s_last_vote_tick = 0U;
static uint32_t s_last_stack_check_tick = 0U;
static uint32_t s_watchdog_feed_count = 0U;
static uint32_t s_watchdog_missed_vote_count = 0U;
static uint32_t s_stack_total_bytes = 0U;
static uint32_t s_stack_used_bytes = 0U;
static uint32_t s_stack_min_free_bytes = 0U;
static uint8_t s_stack_initialized = 0U;
static uint8_t s_stack_guard_ok = 1U;
static uint8_t s_fatal_recorded = 0U;

static uint32_t runtime_crc32_update(uint32_t crc, const uint8_t *data,
                                     uint32_t length)
{
    uint32_t index;
    uint32_t bit;

    for (index = 0U; index < length; ++index)
    {
        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 1U) != 0U) ?
                  ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc;
}

static uint32_t runtime_fault_crc(const RuntimeMonitorFaultRecord *record)
{
    static const uint8_t zero_crc[sizeof(uint32_t)] = {0U};
    const uint8_t *bytes;
    const uint32_t crc_offset = (uint32_t)offsetof(RuntimeMonitorFaultRecord,
                                                   crc32);
    uint32_t crc;

    if (record == NULL)
    {
        return 0U;
    }
    bytes = (const uint8_t *)record;
    crc = runtime_crc32_update(0xFFFFFFFFUL, bytes, crc_offset);
    crc = runtime_crc32_update(crc, zero_crc, sizeof(zero_crc));
    crc = runtime_crc32_update(crc,
                               &bytes[crc_offset + sizeof(uint32_t)],
                               (uint32_t)sizeof(*record) - crc_offset -
                               (uint32_t)sizeof(uint32_t));
    return crc ^ 0xFFFFFFFFUL;
}

static uint8_t runtime_fault_record_valid(const RuntimeMonitorFaultRecord *record)
{
    if ((record == NULL) ||
        (record->magic != RUNTIME_MONITOR_FAULT_MAGIC) ||
        (record->version != RUNTIME_MONITOR_FAULT_VERSION) ||
        (record->size != sizeof(RuntimeMonitorFaultRecord)))
    {
        return 0U;
    }
    return (record->crc32 == runtime_fault_crc(record)) ? 1U : 0U;
}

static uint8_t runtime_stack_frame_valid(const uint32_t *stacked_sp)
{
    uintptr_t address = (uintptr_t)stacked_sp;
    uintptr_t end = address + (8U * sizeof(uint32_t));

    if ((stacked_sp == NULL) || (end < address))
    {
        return 0U;
    }
    if (((address >= 0x20000000UL) && (end <= 0x20020000UL)) ||
        ((address >= 0x24000000UL) && (end <= 0x24080000UL)) ||
        ((address >= 0x30000000UL) && (end <= 0x30048000UL)) ||
        ((address >= 0x38000000UL) && (end <= 0x38010000UL)))
    {
        return 1U;
    }
    return 0U;
}

static void runtime_backup_enable(void)
{
    __HAL_RCC_BKPRAM_CLK_ENABLE();
    __DSB();
}

static void runtime_fault_store(const RuntimeMonitorFaultRecord *record)
{
    volatile RuntimeMonitorFaultRecord *destination =
        (volatile RuntimeMonitorFaultRecord *)RUNTIME_MONITOR_BACKUP_ADDRESS;
    const uint32_t *source = (const uint32_t *)record;
    volatile uint32_t *target = (volatile uint32_t *)destination;
    uint32_t index;

    runtime_backup_enable();
    for (index = 0U; index < (sizeof(*record) / sizeof(uint32_t)); ++index)
    {
        target[index] = source[index];
    }
    __DSB();
    SCB_CleanDCache_by_Addr((uint32_t *)RUNTIME_MONITOR_BACKUP_ADDRESS,
                            (int32_t)sizeof(*record));
    __DSB();
}

static void runtime_stack_measure(void)
{
    uint32_t *cursor;
    uint32_t *bottom = (uint32_t *)((uintptr_t)&_estack -
                                   (uintptr_t)&_Min_Stack_Size);
    uint32_t *top = (uint32_t *)&_estack;
    uint32_t free_bytes;
    uint32_t index;

    if (s_stack_initialized == 0U)
    {
        return;
    }

    s_stack_guard_ok = 1U;
    for (index = 0U; index < RUNTIME_MONITOR_STACK_GUARD_WORDS; ++index)
    {
        if (bottom[index] != RUNTIME_MONITOR_STACK_PATTERN)
        {
            s_stack_guard_ok = 0U;
            break;
        }
    }

    cursor = bottom;
    while ((cursor < top) && (*cursor == RUNTIME_MONITOR_STACK_PATTERN))
    {
        cursor++;
    }
    free_bytes = (uint32_t)((uintptr_t)cursor - (uintptr_t)bottom);
    s_stack_used_bytes = s_stack_total_bytes - free_bytes;
    if (free_bytes < s_stack_min_free_bytes)
    {
        s_stack_min_free_bytes = free_bytes;
    }
}

static void runtime_watchdog_refresh(uint32_t now)
{
    if ((s_watchdog != NULL) && (HAL_IWDG_Refresh(s_watchdog) == HAL_OK))
    {
        s_watchdog_feed_count++;
        s_last_feed_tick = now;
    }
}

static void runtime_monitor_process(void)
{
    uint32_t now = HAL_GetTick();

    if ((s_stack_initialized != 0U) &&
        ((now - s_last_stack_check_tick) >= RUNTIME_MONITOR_STACK_CHECK_PERIOD_MS))
    {
        s_last_stack_check_tick = now;
        runtime_stack_measure();
        if ((s_stack_guard_ok == 0U) && (s_fatal_recorded == 0U))
        {
            RuntimeMonitor_CaptureFaultAndReset(RUNTIME_FAULT_STACK_GUARD,
                                                NULL, 0U);
        }
    }

    if ((s_watchdog == NULL) ||
        ((now - s_last_vote_tick) < RUNTIME_MONITOR_FEED_PERIOD_MS))
    {
        return;
    }
    s_last_vote_tick = now;

    if (s_state == RUNTIME_MONITOR_STATE_BOOT)
    {
        runtime_watchdog_refresh(now);
        return;
    }
    if (s_state != RUNTIME_MONITOR_STATE_RUNNING)
    {
        return;
    }

    if (((now - s_last_control_tick) >= RUNTIME_MONITOR_CONTROL_STALL_MS) &&
        (s_fatal_recorded == 0U))
    {
        s_watchdog_missed_vote_count++;
        RuntimeMonitor_CaptureFaultAndReset(RUNTIME_FAULT_CONTROL_STALL,
                                            NULL, 0U);
    }

    if ((s_foreground_sequence != s_fed_foreground_sequence) &&
        (s_control_sequence != s_fed_control_sequence) &&
        (s_stack_guard_ok != 0U))
    {
        runtime_watchdog_refresh(now);
        s_fed_foreground_sequence = s_foreground_sequence;
        s_fed_control_sequence = s_control_sequence;
    }
    else
    {
        s_watchdog_missed_vote_count++;
    }
}

static void runtime_uart4_stop_best_effort(void)
{
    static const uint8_t stop_command[] = "Speed:0\n";
    uint32_t index;

    if (((RCC->APB1LENR & RCC_APB1LENR_UART4EN) == 0U) ||
        ((UART4->CR1 & (USART_CR1_UE | USART_CR1_TE)) !=
         (USART_CR1_UE | USART_CR1_TE)))
    {
        return;
    }

    for (index = 0U; index < (sizeof(stop_command) - 1U); ++index)
    {
        uint32_t timeout = 120000U;
        while (((UART4->ISR & USART_ISR_TXE_TXFNF) == 0U) && (timeout != 0U))
        {
            timeout--;
        }
        if (timeout == 0U)
        {
            return;
        }
        UART4->TDR = stop_command[index];
    }
    {
        uint32_t timeout = 480000U;
        while (((UART4->ISR & USART_ISR_TC) == 0U) && (timeout != 0U))
        {
            timeout--;
        }
    }
}

void RuntimeMonitor_EarlyInit(void)
{
    volatile const RuntimeMonitorFaultRecord *source;

    s_reset_flags = RCC->RSR;
    runtime_backup_enable();
    source = (volatile const RuntimeMonitorFaultRecord *)
             RUNTIME_MONITOR_BACKUP_ADDRESS;
    memcpy(&s_last_fault, (const void *)source, sizeof(s_last_fault));
    if (runtime_fault_record_valid(&s_last_fault) == 0U)
    {
        memset(&s_last_fault, 0, sizeof(s_last_fault));
    }
    __HAL_RCC_CLEAR_RESET_FLAGS();

    s_state = RUNTIME_MONITOR_STATE_EARLY;
    s_last_foreground_tick = HAL_GetTick();
    s_last_control_tick = HAL_GetTick();
}

void RuntimeMonitor_AttachWatchdog(IWDG_HandleTypeDef *hiwdg)
{
    s_watchdog = hiwdg;
    s_state = RUNTIME_MONITOR_STATE_BOOT;
    s_last_feed_tick = HAL_GetTick();
    s_last_vote_tick = s_last_feed_tick;
    runtime_watchdog_refresh(s_last_feed_tick);
}

void RuntimeMonitor_BootProgress(void)
{
    s_foreground_sequence++;
    s_last_foreground_tick = HAL_GetTick();
    runtime_monitor_process();
}

void RuntimeMonitor_StackInit(void)
{
    uint32_t *cursor = (uint32_t *)((uintptr_t)&_estack -
                                   (uintptr_t)&_Min_Stack_Size);
    uint32_t *top = (uint32_t *)&_estack;
    uintptr_t current_msp = (uintptr_t)__get_MSP();
    uintptr_t fill_end = (current_msp > 96U) ? (current_msp - 96U) : current_msp;

    s_stack_total_bytes = (uint32_t)((uintptr_t)top - (uintptr_t)cursor);
    while ((cursor < top) && (((uintptr_t)cursor + sizeof(uint32_t)) <= fill_end))
    {
        *cursor++ = RUNTIME_MONITOR_STACK_PATTERN;
    }
    __DMB();
    s_stack_min_free_bytes = s_stack_total_bytes;
    s_stack_initialized = 1U;
    s_stack_guard_ok = 1U;
    s_last_stack_check_tick = HAL_GetTick();
    runtime_stack_measure();
}

void RuntimeMonitor_StartRuntime(void)
{
    uint32_t now = HAL_GetTick();

    s_state = RUNTIME_MONITOR_STATE_RUNNING;
    s_fed_foreground_sequence = s_foreground_sequence;
    s_fed_control_sequence = s_control_sequence;
    s_last_foreground_tick = now;
    s_last_control_tick = now;
    s_last_vote_tick = now;
    runtime_watchdog_refresh(now);
}

void RuntimeMonitor_MainLoopHeartbeat(void)
{
    RuntimeMonitor_ForegroundHeartbeat();
}

void RuntimeMonitor_ForegroundHeartbeat(void)
{
    s_foreground_sequence++;
    s_last_foreground_tick = HAL_GetTick();
    runtime_monitor_process();
}

void RuntimeMonitor_ControlTickFromISR(void)
{
    s_control_sequence++;
    s_last_control_tick = HAL_GetTick();
}

void RuntimeMonitor_GetSnapshot(RuntimeMonitorSnapshot *snapshot)
{
    uint32_t now;

    if (snapshot == NULL)
    {
        return;
    }
    now = HAL_GetTick();
    runtime_stack_measure();
    memset(snapshot, 0, sizeof(*snapshot));
    snapshot->reset_flags = s_reset_flags;
    snapshot->state = s_state;
    snapshot->watchdog_feed_count = s_watchdog_feed_count;
    snapshot->watchdog_missed_vote_count = s_watchdog_missed_vote_count;
    snapshot->last_feed_age_ms = now - s_last_feed_tick;
    snapshot->foreground_age_ms = now - s_last_foreground_tick;
    snapshot->control_age_ms = now - s_last_control_tick;
    snapshot->stack_total_bytes = s_stack_total_bytes;
    snapshot->stack_used_bytes = s_stack_used_bytes;
    snapshot->stack_min_free_bytes = s_stack_min_free_bytes;
    snapshot->stack_guard_ok = s_stack_guard_ok;
    snapshot->last_fault_valid = runtime_fault_record_valid(&s_last_fault);
    if (snapshot->last_fault_valid != 0U)
    {
        snapshot->last_fault_type = (RuntimeFaultType)s_last_fault.fault_type;
        snapshot->last_fault_pc = s_last_fault.stacked_pc;
        snapshot->last_fault_lr = s_last_fault.stacked_lr;
        snapshot->last_fault_cfsr = s_last_fault.cfsr;
        snapshot->last_fault_hfsr = s_last_fault.hfsr;
        snapshot->last_fault_bfar = s_last_fault.bfar;
        snapshot->last_fault_mmfar = s_last_fault.mmfar;
    }
}

const char *RuntimeMonitor_ResetReasonText(uint32_t reset_flags)
{
    if ((reset_flags & RCC_RSR_IWDG1RSTF) != 0U) return "IWDG";
    if ((reset_flags & RCC_RSR_WWDG1RSTF) != 0U) return "WWDG";
    if ((reset_flags & RCC_RSR_SFTRSTF) != 0U) return "SOFTWARE";
    if ((reset_flags & RCC_RSR_PORRSTF) != 0U) return "POWER ON";
    if ((reset_flags & RCC_RSR_BORRSTF) != 0U) return "BROWNOUT";
    if ((reset_flags & RCC_RSR_PINRSTF) != 0U) return "RESET PIN";
    if ((reset_flags & RCC_RSR_LPWRRSTF) != 0U) return "LOW POWER";
    if ((reset_flags & RCC_RSR_CPURSTF) != 0U) return "CPU RESET";
    return "UNKNOWN";
}

const char *RuntimeMonitor_FaultTypeText(RuntimeFaultType fault_type)
{
    switch (fault_type)
    {
        case RUNTIME_FAULT_HARD: return "HARDFAULT";
        case RUNTIME_FAULT_MEMMANAGE: return "MEMMANAGE";
        case RUNTIME_FAULT_BUS: return "BUSFAULT";
        case RUNTIME_FAULT_USAGE: return "USAGEFAULT";
        case RUNTIME_FAULT_ERROR_HANDLER: return "ERROR HANDLER";
        case RUNTIME_FAULT_STACK_GUARD: return "STACK GUARD";
        case RUNTIME_FAULT_CONTROL_STALL: return "CONTROL STALL";
        default: return "NONE";
    }
}

void RuntimeMonitor_SafeShutdownImmediate(void)
{
    __disable_irq();
    __DSB();

    TIM1->CCR1 = 0U;
    TIM1->CCR2 = 0U;
    TIM1->CCR3 = 0U;
    TIM1->CCR4 = 0U;
    TIM1->CCER &= ~(TIM_CCER_CC1E | TIM_CCER_CC2E |
                    TIM_CCER_CC3E | TIM_CCER_CC4E);
    TIM1->BDTR &= ~TIM_BDTR_MOE;
    GPIOG->BSRR = ((uint32_t)(M1_PH_Pin | M2_PH_Pin |
                              M3_PH_Pin | M4_PH_Pin) << 16U);
    runtime_uart4_stop_best_effort();
    __DSB();
}

void RuntimeMonitor_RecordFatal(RuntimeFaultType fault_type,
                                const uint32_t *stacked_sp,
                                uint32_t exception_lr)
{
    RuntimeMonitorFaultRecord *record = &s_fault_work;
    const uint32_t *basic_frame = stacked_sp;
    uint8_t stack_valid;

    if (s_fatal_recorded != 0U)
    {
        return;
    }
    s_fatal_recorded = 1U;
    s_state = RUNTIME_MONITOR_STATE_FATAL;
    RuntimeMonitor_SafeShutdownImmediate();

    memset(record, 0, sizeof(*record));
    record->magic = RUNTIME_MONITOR_FAULT_MAGIC;
    record->version = RUNTIME_MONITOR_FAULT_VERSION;
    record->size = (uint16_t)sizeof(*record);
    record->fault_type = (uint32_t)fault_type;
    record->reset_flags = RCC->RSR;
    record->tick_ms = HAL_GetTick();
    record->exception_lr = exception_lr;
    record->cfsr = SCB->CFSR;
    record->hfsr = SCB->HFSR;
    record->dfsr = SCB->DFSR;
    record->afsr = SCB->AFSR;
    record->bfar = SCB->BFAR;
    record->mmfar = SCB->MMFAR;
    record->msp = __get_MSP();
    record->psp = __get_PSP();
    record->control_sequence = s_control_sequence;
    record->foreground_sequence = s_foreground_sequence;
    /* Avoid retaining an old IWDG marker when the fatal path is immediately
     * followed by a software reset. The persistent record is authoritative. */
    __HAL_RCC_CLEAR_RESET_FLAGS();

    /* EXC_RETURN bit 4 is clear when an extended floating-point frame was
     * stacked. The basic R0..xPSR frame then starts after 18 FP words. */
    if ((basic_frame != NULL) && ((exception_lr & (1UL << 4U)) == 0U))
    {
        basic_frame += 18U;
    }
    stack_valid = runtime_stack_frame_valid(basic_frame);
    if (stack_valid != 0U)
    {
        record->stacked_r0 = basic_frame[0];
        record->stacked_r1 = basic_frame[1];
        record->stacked_r2 = basic_frame[2];
        record->stacked_r3 = basic_frame[3];
        record->stacked_r12 = basic_frame[4];
        record->stacked_lr = basic_frame[5];
        record->stacked_pc = basic_frame[6];
        record->stacked_psr = basic_frame[7];
    }
    record->crc32 = runtime_fault_crc(record);
    runtime_fault_store(record);
    s_last_fault = *record;
}

void RuntimeMonitor_CaptureFaultAndReset(RuntimeFaultType fault_type,
                                         const uint32_t *stacked_sp,
                                         uint32_t exception_lr)
{
    RuntimeMonitor_RecordFatal(fault_type, stacked_sp, exception_lr);
    __DSB();
    NVIC_SystemReset();
    while (1)
    {
    }
}
