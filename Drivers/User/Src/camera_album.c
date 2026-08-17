#include "camera_album.h"

#include "fatfs.h"
#include "ff.h"

#include <ctype.h>
#include <stdio.h>
#include <string.h>

#define CAMERA_ALBUM_WRITE_CHUNK 4096U
#define CAMERA_ALBUM_FIRST_INDEX 1U
#define CAMERA_ALBUM_LAST_INDEX  99999U
#define CAMERA_ALBUM_TEMP_PATH   CAMERA_ALBUM_DIRECTORY "/CAPTURE.TMP"

static uint8_t s_last_fs_error = (uint8_t)FR_OK;
/* FatFs can forward full sectors directly to SDMMC. Stage camera data between
 * D2 SRAM and the DMA-proven AXI SRAM domain for both reads and writes. */
static uint8_t s_io_buffer[CAMERA_ALBUM_WRITE_CHUNK]
  __attribute__((section(".media_pool"), aligned(32)));
static uint32_t s_last_write_offset = 0U;
static uint32_t s_last_write_request = 0U;
static uint32_t s_last_write_actual = 0U;
static uint32_t s_last_read_offset = 0U;
static uint32_t s_last_read_request = 0U;
static uint32_t s_last_read_actual = 0U;

static void camera_album_unmount(void)
{
  (void)f_mount(NULL, (TCHAR const *)SDPath, 1U);
}

static int8_t camera_album_mount(void)
{
  FRESULT fr = f_mount(&SDFatFS, (TCHAR const *)SDPath, 1U);

  s_last_fs_error = (uint8_t)fr;
  return (fr == FR_OK) ? CAMERA_ALBUM_OK : CAMERA_ALBUM_ERR_MOUNT;
}

static int8_t camera_album_ensure_directory_mounted(void)
{
  FRESULT fr;

  s_last_fs_error = (uint8_t)FR_OK;

  fr = f_mkdir("/DCIM");
  if ((fr != FR_OK) && (fr != FR_EXIST))
  {
    s_last_fs_error = (uint8_t)fr;
    return CAMERA_ALBUM_ERR_DIRECTORY;
  }

  fr = f_mkdir(CAMERA_ALBUM_DIRECTORY);
  if ((fr != FR_OK) && (fr != FR_EXIST))
  {
    s_last_fs_error = (uint8_t)fr;
    return CAMERA_ALBUM_ERR_DIRECTORY;
  }

  return CAMERA_ALBUM_OK;
}

static uint8_t camera_album_parse_index(const char *name, uint32_t *index)
{
  uint32_t value = 0U;
  uint8_t i;

  if ((name == NULL) || (index == NULL) || (strlen(name) != 12U) ||
      (toupper((unsigned char)name[0]) != 'I') ||
      (toupper((unsigned char)name[1]) != 'M') ||
      (toupper((unsigned char)name[2]) != 'G') ||
      (name[8] != '.') ||
      (toupper((unsigned char)name[9]) != 'J') ||
      (toupper((unsigned char)name[10]) != 'P') ||
      (toupper((unsigned char)name[11]) != 'G'))
  {
    return 0U;
  }

  for (i = 3U; i < 8U; ++i)
  {
    if ((name[i] < '0') || (name[i] > '9'))
    {
      return 0U;
    }
    value = (value * 10U) + (uint32_t)(name[i] - '0');
  }

  *index = value;
  return 1U;
}

static int8_t camera_album_next_path(char *path, size_t path_size)
{
  DIR dir;
  FILINFO info;
  FRESULT fr;
  uint32_t max_index = 0U;
  uint32_t parsed_index;
  uint32_t candidate;
  uint32_t first_candidate;
  int written;

  fr = f_opendir(&dir, CAMERA_ALBUM_DIRECTORY);
  if (fr != FR_OK)
  {
    s_last_fs_error = (uint8_t)fr;
    return CAMERA_ALBUM_ERR_DIRECTORY;
  }

  for (;;)
  {
    fr = f_readdir(&dir, &info);
    if ((fr != FR_OK) || (info.fname[0] == '\0'))
    {
      break;
    }
    if (((info.fattrib & AM_DIR) == 0U) &&
        (camera_album_parse_index(info.fname, &parsed_index) != 0U) &&
        (parsed_index > max_index))
    {
      max_index = parsed_index;
    }
  }
  (void)f_closedir(&dir);
  if (fr != FR_OK)
  {
    s_last_fs_error = (uint8_t)fr;
    return CAMERA_ALBUM_ERR_NAME;
  }

  candidate = max_index + 1U;
  if ((candidate < CAMERA_ALBUM_FIRST_INDEX) ||
      (candidate > CAMERA_ALBUM_LAST_INDEX))
  {
    candidate = CAMERA_ALBUM_FIRST_INDEX;
  }
  first_candidate = candidate;

  /* After IMG99999.JPG, find the first reusable gap without overwriting an
     existing photo. Normal operation takes the fast path above. */
  for (;;)
  {
    FILINFO existing;

    written = snprintf(path, path_size, CAMERA_ALBUM_DIRECTORY "/IMG%05lu.JPG",
                       (unsigned long)candidate);
    if ((written <= 0) || ((size_t)written >= path_size))
    {
      return CAMERA_ALBUM_ERR_NAME;
    }

    fr = f_stat(path, &existing);
    if (fr == FR_NO_FILE)
    {
      return CAMERA_ALBUM_OK;
    }
    if (fr != FR_OK)
    {
      s_last_fs_error = (uint8_t)fr;
      return CAMERA_ALBUM_ERR_NAME;
    }

    candidate++;
    if (candidate > CAMERA_ALBUM_LAST_INDEX)
    {
      candidate = CAMERA_ALBUM_FIRST_INDEX;
    }
    if (candidate == first_candidate)
    {
      return CAMERA_ALBUM_ERR_NAME;
    }
  }
}

int8_t CameraAlbum_EnsureDirectory(void)
{
  int8_t result = camera_album_mount();

  if (result == CAMERA_ALBUM_OK)
  {
    result = camera_album_ensure_directory_mounted();
  }
  camera_album_unmount();
  return result;
}

int8_t CameraAlbum_SaveJpeg(const uint8_t *jpeg_data,
                            uint32_t jpeg_size,
                            char *saved_path,
                            size_t saved_path_size)
{
  FIL file;
  FRESULT fr;
  UINT written;
  uint32_t offset = 0U;
  int8_t result;
  char final_path[CAMERA_ALBUM_PATH_MAX];

  if ((jpeg_data == NULL) || (jpeg_size < 4U) ||
      (saved_path == NULL) || (saved_path_size == 0U) ||
      (jpeg_data[0] != 0xFFU) || (jpeg_data[1] != 0xD8U))
  {
    return CAMERA_ALBUM_ERR_PARAM;
  }
  saved_path[0] = '\0';
  s_last_fs_error = (uint8_t)FR_OK;
  s_last_write_offset = 0U;
  s_last_write_request = 0U;
  s_last_write_actual = 0U;

  result = camera_album_mount();
  if (result != CAMERA_ALBUM_OK)
  {
    return result;
  }
  result = camera_album_ensure_directory_mounted();
  if (result != CAMERA_ALBUM_OK)
  {
    camera_album_unmount();
    return result;
  }
  result = camera_album_next_path(final_path, sizeof(final_path));
  if (result != CAMERA_ALBUM_OK)
  {
    camera_album_unmount();
    return result;
  }

  (void)f_unlink(CAMERA_ALBUM_TEMP_PATH);
  fr = f_open(&file, CAMERA_ALBUM_TEMP_PATH, FA_CREATE_ALWAYS | FA_WRITE);
  if (fr != FR_OK)
  {
    s_last_fs_error = (uint8_t)fr;
    camera_album_unmount();
    return CAMERA_ALBUM_ERR_OPEN;
  }

  result = CAMERA_ALBUM_OK;
  while (offset < jpeg_size)
  {
    uint32_t chunk = jpeg_size - offset;
    if (chunk > CAMERA_ALBUM_WRITE_CHUNK)
    {
      chunk = CAMERA_ALBUM_WRITE_CHUNK;
    }

    memcpy(s_io_buffer, &jpeg_data[offset], chunk);
    s_last_write_offset = offset;
    s_last_write_request = chunk;
    written = 0U;
    fr = f_write(&file, s_io_buffer, (UINT)chunk, &written);
    s_last_write_actual = written;
    if ((fr != FR_OK) || (written != (UINT)chunk))
    {
      s_last_fs_error = (uint8_t)fr;
      result = CAMERA_ALBUM_ERR_WRITE;
      break;
    }
    offset += chunk;
  }

  if (result == CAMERA_ALBUM_OK)
  {
    fr = f_sync(&file);
    if (fr != FR_OK)
    {
      s_last_fs_error = (uint8_t)fr;
      result = CAMERA_ALBUM_ERR_SYNC;
    }
  }

  fr = f_close(&file);
  if ((result == CAMERA_ALBUM_OK) && (fr != FR_OK))
  {
    s_last_fs_error = (uint8_t)fr;
    result = CAMERA_ALBUM_ERR_SYNC;
  }

  if (result == CAMERA_ALBUM_OK)
  {
    fr = f_rename(CAMERA_ALBUM_TEMP_PATH, final_path);
    if (fr != FR_OK)
    {
      s_last_fs_error = (uint8_t)fr;
      result = CAMERA_ALBUM_ERR_RENAME;
    }
  }

  if (result != CAMERA_ALBUM_OK)
  {
    (void)f_unlink(CAMERA_ALBUM_TEMP_PATH);
  }
  else
  {
    int copied = snprintf(saved_path, saved_path_size, "%s", final_path);
    if ((copied <= 0) || ((size_t)copied >= saved_path_size))
    {
      result = CAMERA_ALBUM_ERR_NAME;
    }
  }

  camera_album_unmount();
  return result;
}

int8_t CameraAlbum_LoadJpeg(const char *path,
                            uint8_t *buffer,
                            uint32_t capacity,
                            uint32_t *jpeg_size)
{
  FIL file;
  FRESULT fr;
  UINT read_count;
  uint32_t file_size;
  uint32_t offset = 0U;
  int8_t result;

  if ((path == NULL) || (path[0] == '\0') || (buffer == NULL) ||
      (capacity < 4U) || (jpeg_size == NULL))
  {
    return CAMERA_ALBUM_ERR_PARAM;
  }
  *jpeg_size = 0U;
  s_last_fs_error = (uint8_t)FR_OK;
  s_last_read_offset = 0U;
  s_last_read_request = 0U;
  s_last_read_actual = 0U;

  result = camera_album_mount();
  if (result != CAMERA_ALBUM_OK)
  {
    return result;
  }

  fr = f_open(&file, path, FA_READ);
  if (fr != FR_OK)
  {
    s_last_fs_error = (uint8_t)fr;
    camera_album_unmount();
    return CAMERA_ALBUM_ERR_OPEN;
  }

  file_size = (uint32_t)f_size(&file);
  if ((file_size < 4U) || (file_size > capacity))
  {
    result = (file_size > capacity) ? CAMERA_ALBUM_ERR_TOO_LARGE :
                                      CAMERA_ALBUM_ERR_FORMAT;
  }
  else
  {
    result = CAMERA_ALBUM_OK;
    while (offset < file_size)
    {
      uint32_t chunk = file_size - offset;
      if (chunk > CAMERA_ALBUM_WRITE_CHUNK)
      {
        chunk = CAMERA_ALBUM_WRITE_CHUNK;
      }
      s_last_read_offset = offset;
      s_last_read_request = chunk;
      read_count = 0U;
      fr = f_read(&file, s_io_buffer, (UINT)chunk, &read_count);
      s_last_read_actual = read_count;
      if ((fr != FR_OK) || (read_count != (UINT)chunk))
      {
        s_last_fs_error = (uint8_t)fr;
        result = CAMERA_ALBUM_ERR_READ;
        break;
      }
      memcpy(&buffer[offset], s_io_buffer, chunk);
      offset += chunk;
    }
  }

  (void)f_close(&file);
  camera_album_unmount();
  if (result != CAMERA_ALBUM_OK)
  {
    return result;
  }
  if ((buffer[0] != 0xFFU) || (buffer[1] != 0xD8U))
  {
    return CAMERA_ALBUM_ERR_FORMAT;
  }

  *jpeg_size = file_size;
  return CAMERA_ALBUM_OK;
}

uint8_t CameraAlbum_GetLastFsError(void)
{
  return s_last_fs_error;
}

uint32_t CameraAlbum_GetLastWriteOffset(void)
{
  return s_last_write_offset;
}

uint32_t CameraAlbum_GetLastWriteRequest(void)
{
  return s_last_write_request;
}

uint32_t CameraAlbum_GetLastWriteActual(void)
{
  return s_last_write_actual;
}

uint32_t CameraAlbum_GetLastReadOffset(void)
{
  return s_last_read_offset;
}

uint32_t CameraAlbum_GetLastReadRequest(void)
{
  return s_last_read_request;
}

uint32_t CameraAlbum_GetLastReadActual(void)
{
  return s_last_read_actual;
}
