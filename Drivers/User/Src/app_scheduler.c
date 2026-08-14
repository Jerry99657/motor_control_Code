#include "app_scheduler.h"
#include <stddef.h>
#include <string.h>

void AppScheduler_Init(AppScheduler *scheduler)
{
    if (scheduler == NULL)
    {
        return;
    }

    memset(scheduler, 0, sizeof(*scheduler));
}

uint8_t AppScheduler_Add(AppScheduler *scheduler,
                         AppSchedulerTaskFn function,
                         void *context,
                         uint32_t period_ms,
                         uint32_t first_run_ms)
{
    AppSchedulerTask *task;

    if ((scheduler == NULL) || (function == NULL) ||
        (scheduler->task_count >= APP_SCHEDULER_MAX_TASKS))
    {
        return 0U;
    }

    task = &scheduler->tasks[scheduler->task_count++];
    task->function = function;
    task->context = context;
    task->period_ms = period_ms;
    task->next_run_ms = first_run_ms;
    task->enabled = 1U;
    return 1U;
}

void AppScheduler_Run(AppScheduler *scheduler, uint32_t now)
{
    uint8_t index;

    if (scheduler == NULL)
    {
        return;
    }

    scheduler->run_count++;

    for (index = 0U; index < scheduler->task_count; ++index)
    {
        AppSchedulerTask *task = &scheduler->tasks[index];

        if ((task->enabled == 0U) || (task->function == NULL))
        {
            continue;
        }

        if ((task->period_ms == 0U) ||
            ((int32_t)(now - task->next_run_ms) >= 0))
        {
            if (task->period_ms != 0U)
            {
                task->next_run_ms = now + task->period_ms;
            }
            task->function(now, task->context);
            scheduler->task_call_count++;
        }
    }
}

void AppScheduler_GetStats(const AppScheduler *scheduler,
                           AppSchedulerStats *stats)
{
    if ((scheduler == NULL) || (stats == NULL))
    {
        return;
    }

    stats->run_count = scheduler->run_count;
    stats->task_call_count = scheduler->task_call_count;
    stats->task_count = scheduler->task_count;
}
