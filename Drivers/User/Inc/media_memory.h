#ifndef MEDIA_MEMORY_H
#define MEDIA_MEMORY_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
  MEDIA_MEMORY_OWNER_NONE = 0,
  MEDIA_MEMORY_OWNER_MJPEG,
  MEDIA_MEMORY_OWNER_SD_ANIM,
  MEDIA_MEMORY_OWNER_QSPI_ANIM
} MediaMemoryOwner;

uint8_t *MediaMemory_Acquire(MediaMemoryOwner owner, uint32_t required_bytes, uint32_t *capacity_bytes);
void MediaMemory_Release(MediaMemoryOwner owner);
MediaMemoryOwner MediaMemory_GetOwner(void);
uint32_t MediaMemory_GetCapacity(void);

#ifdef __cplusplus
}
#endif

#endif /* MEDIA_MEMORY_H */
