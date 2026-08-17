#ifndef APP_RUNTIME_H
#define APP_RUNTIME_H

#include "app_context.h"
#include "app_scheduler.h"

void AppRuntime_EarlyInit(void);
void AppRuntime_Init(const AppContext *context);
void AppRuntime_Process(void);
uint32_t AppRuntime_GetIdleDelayMs(void);
void AppRuntime_GetSchedulerStats(AppSchedulerStats *stats);

#endif /* APP_RUNTIME_H */
