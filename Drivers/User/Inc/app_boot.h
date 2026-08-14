#ifndef APP_BOOT_H
#define APP_BOOT_H

#include <stdint.h>

void AppBoot_EnableCache(void);
void AppBoot_Run(void);
void AppBoot_ProcessPreUi(void);
void AppBoot_ProcessPostUi(void);
uint8_t AppBoot_IsCdcReady(void);

/* Boot/media diagnostics use the same buffered CDC logger. */
void Boot_DebugStageLog(const char *text);
void Boot_DebugFlushStageLogsViaCdc(void);

#endif /* APP_BOOT_H */
