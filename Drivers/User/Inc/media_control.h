#ifndef MEDIA_CONTROL_H
#define MEDIA_CONTROL_H

#include <stdint.h>

typedef enum
{
    MEDIA_CONTROL_NONE = 0,
    MEDIA_CONTROL_STOP,
    MEDIA_CONTROL_BACK,
    MEDIA_CONTROL_PAUSE_CHANGED,
    MEDIA_CONTROL_SEEK_BACK,
    MEDIA_CONTROL_SEEK_FORWARD
} media_control_action_t;

void MediaControl_Init(void);
media_control_action_t MediaControl_Poll(void);
uint8_t MediaControl_IsPaused(void);
uint8_t MediaControl_IsSeekHeld(void);
void MediaControl_ShowPausedHud(void);

#endif /* MEDIA_CONTROL_H */
