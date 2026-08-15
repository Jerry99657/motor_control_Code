#ifndef LVGL_APP_H
#define LVGL_APP_H

#include "ui_navigation.h"
#include <stdint.h>

void LVGL_App_Init(void);
void LVGL_App_Process(void);
void LVGL_App_GetNavigationSnapshot(UiNavigationSnapshot *snapshot);

#endif /* LVGL_APP_H */
