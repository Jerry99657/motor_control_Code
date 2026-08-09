#ifndef MJPEG_PLAYER_H
#define MJPEG_PLAYER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define MJPEG_PLAYER_OK                   0
#define MJPEG_PLAYER_ERR_PARAM           -1
#define MJPEG_PLAYER_ERR_MOUNT           -2
#define MJPEG_PLAYER_ERR_FILE            -3
#define MJPEG_PLAYER_ERR_FORMAT          -4
#define MJPEG_PLAYER_ERR_IO              -5
#define MJPEG_PLAYER_ERR_DECODE          -6
#define MJPEG_PLAYER_ERR_STOPPED         -7
#define MJPEG_PLAYER_ERR_UNSUPPORTED     -8
#define MJPEG_PLAYER_ERR_FRAME_TOO_LARGE -9
#define MJPEG_PLAYER_ERR_BUSY            -10
#define MJPEG_PLAYER_ERR_BACK            -11

int8_t MJPEG_Player_PlayFile(const char *file_path);
uint8_t MJPEG_Player_GetLastFsError(void);
int8_t MJPEG_Player_DecodeMemoryToRgb565(uint8_t *jpeg_data,
                                        uint32_t jpeg_len,
                                        uint32_t jpeg_capacity,
                                        uint16_t *rgb565_buffer,
                                        uint32_t rgb565_capacity_bytes,
                                        uint16_t *image_width,
                                        uint16_t *image_height);

#ifdef __cplusplus
}
#endif

#endif /* MJPEG_PLAYER_H */
