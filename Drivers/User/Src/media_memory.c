#include "media_memory.h"

#include "lcd_spi_154.h"
#include "main.h"

#define MEDIA_MEMORY_FRAME_BYTES ((uint32_t)LCD_Width * (uint32_t)LCD_Height * 2U)
#define MEDIA_MEMORY_CAMERA_BYTES (320U * 240U * 2U)
#define MEDIA_MEMORY_POOL_BYTES \
  ((MEDIA_MEMORY_FRAME_BYTES > MEDIA_MEMORY_CAMERA_BYTES) \
    ? MEDIA_MEMORY_FRAME_BYTES : MEDIA_MEMORY_CAMERA_BYTES)

static uint8_t s_media_frame_pool[MEDIA_MEMORY_POOL_BYTES]
  __attribute__((section(".media_pool"), aligned(32)));
static volatile MediaMemoryOwner s_media_owner = MEDIA_MEMORY_OWNER_NONE;

uint8_t *MediaMemory_Acquire(MediaMemoryOwner owner, uint32_t required_bytes, uint32_t *capacity_bytes)
{
  uint32_t primask;
  uint8_t *buffer = NULL;

  if (capacity_bytes != NULL)
  {
    *capacity_bytes = sizeof(s_media_frame_pool);
  }

  if ((owner == MEDIA_MEMORY_OWNER_NONE) || (required_bytes == 0U) ||
      (required_bytes > sizeof(s_media_frame_pool)))
  {
    return NULL;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (s_media_owner == MEDIA_MEMORY_OWNER_NONE)
  {
    s_media_owner = owner;
    buffer = s_media_frame_pool;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }

  return buffer;
}

void MediaMemory_Release(MediaMemoryOwner owner)
{
  uint32_t primask;

  if (owner == MEDIA_MEMORY_OWNER_NONE)
  {
    return;
  }

  primask = __get_PRIMASK();
  __disable_irq();
  if (s_media_owner == owner)
  {
    __DMB();
    s_media_owner = MEDIA_MEMORY_OWNER_NONE;
  }
  if (primask == 0U)
  {
    __enable_irq();
  }
}

MediaMemoryOwner MediaMemory_GetOwner(void)
{
  return s_media_owner;
}

uint32_t MediaMemory_GetCapacity(void)
{
  return sizeof(s_media_frame_pool);
}
