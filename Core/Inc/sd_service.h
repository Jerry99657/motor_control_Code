#ifndef SD_SERVICE_H
#define SD_SERVICE_H

#include "ff.h"
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct
{
  uint32_t elapsed_ms;
  uint32_t bytes;
  uint32_t bytes_per_sec;
  FRESULT fresult;
} sd_bench_result_t;

FRESULT SD_Service_WriteTextFile(const char *file_name, const char *text);
FRESULT SD_Service_ReadTextFile(const char *file_name, char *out_buf, uint32_t out_buf_size, uint32_t *out_len);
FRESULT SD_Service_DeleteFile(const char *file_name);

FRESULT SD_Service_BenchSequentialWrite(const char *file_name, uint32_t total_bytes, uint32_t chunk_bytes, sd_bench_result_t *result);
FRESULT SD_Service_BenchRandomRead(const char *file_name, uint32_t read_count, uint32_t chunk_bytes, sd_bench_result_t *result);

#ifdef __cplusplus
}
#endif

#endif
