#ifndef CAMERA_ALBUM_H
#define CAMERA_ALBUM_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CAMERA_ALBUM_DIRECTORY "/DCIM/CAMERA"
#define CAMERA_ALBUM_PATH_MAX  64U

#define CAMERA_ALBUM_OK             0
#define CAMERA_ALBUM_ERR_PARAM     -1
#define CAMERA_ALBUM_ERR_MOUNT     -2
#define CAMERA_ALBUM_ERR_DIRECTORY -3
#define CAMERA_ALBUM_ERR_NAME      -4
#define CAMERA_ALBUM_ERR_OPEN      -5
#define CAMERA_ALBUM_ERR_WRITE     -6
#define CAMERA_ALBUM_ERR_SYNC      -7
#define CAMERA_ALBUM_ERR_RENAME    -8
#define CAMERA_ALBUM_ERR_READ      -9
#define CAMERA_ALBUM_ERR_TOO_LARGE -10
#define CAMERA_ALBUM_ERR_FORMAT    -11

int8_t CameraAlbum_EnsureDirectory(void);
int8_t CameraAlbum_SaveJpeg(const uint8_t *jpeg_data,
                            uint32_t jpeg_size,
                            char *saved_path,
                            size_t saved_path_size);
int8_t CameraAlbum_LoadJpeg(const char *path,
                            uint8_t *buffer,
                            uint32_t capacity,
                            uint32_t *jpeg_size);
uint8_t CameraAlbum_GetLastFsError(void);
uint32_t CameraAlbum_GetLastWriteOffset(void);
uint32_t CameraAlbum_GetLastWriteRequest(void);
uint32_t CameraAlbum_GetLastWriteActual(void);
uint32_t CameraAlbum_GetLastReadOffset(void);
uint32_t CameraAlbum_GetLastReadRequest(void);
uint32_t CameraAlbum_GetLastReadActual(void);

#ifdef __cplusplus
}
#endif

#endif /* CAMERA_ALBUM_H */
