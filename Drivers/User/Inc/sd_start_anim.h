#ifndef SD_START_ANIM_H
#define SD_START_ANIM_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define SD_START_ANIM_OK           0
#define SD_START_ANIM_ERR_PARAM   -1
#define SD_START_ANIM_ERR_MOUNT   -2
#define SD_START_ANIM_ERR_FILE    -3
#define SD_START_ANIM_ERR_HEADER  -4
#define SD_START_ANIM_ERR_IO      -5
#define SD_START_ANIM_ERR_STOPPED -6

/* 8.3 filename for FatFs when LFN is disabled. */
#define SD_START_ANIM_FILE_NAME "STARTANI.BIN"

int8_t SD_StartAnim_Play(void);
int8_t SD_StartAnim_PlayFile(const char *file_path);

#ifdef __cplusplus
}
#endif

#endif
