#ifndef APP_SCHEDULER_H
#define APP_SCHEDULER_H

#include <stdint.h>

#define APP_SCHEDULER_MAX_TASKS 12U

typedef void (*AppSchedulerTaskFn)(uint32_t now, void *context);

typedef struct
{
    AppSchedulerTaskFn function;
    void *context;
    uint32_t period_ms;
    uint32_t next_run_ms;
    uint8_t enabled;
} AppSchedulerTask;

typedef struct
{
    AppSchedulerTask tasks[APP_SCHEDULER_MAX_TASKS];
    uint8_t task_count;
    uint32_t run_count;
    uint32_t task_call_count;
} AppScheduler;

typedef struct
{
    uint32_t run_count;
    uint32_t task_call_count;
    uint8_t task_count;
} AppSchedulerStats;

void AppScheduler_Init(AppScheduler *scheduler);
uint8_t AppScheduler_Add(AppScheduler *scheduler,
                         AppSchedulerTaskFn function,
                         void *context,
                         uint32_t period_ms,
                         uint32_t first_run_ms);
void AppScheduler_Run(AppScheduler *scheduler, uint32_t now);
void AppScheduler_GetStats(const AppScheduler *scheduler,
                           AppSchedulerStats *stats);

#endif /* APP_SCHEDULER_H */
