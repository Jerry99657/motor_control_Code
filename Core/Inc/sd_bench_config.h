#ifndef SD_BENCH_CONFIG_H
#define SD_BENCH_CONFIG_H

#include <stdint.h>

/*
 * SD self-test and benchmark centralized configuration.
 * You can override these values in build flags if needed.
 */

#if defined(DEBUG)
#define SD_SELF_TEST_ENABLE_DEFAULT 1U
#else
#define SD_SELF_TEST_ENABLE_DEFAULT 0U
#endif

#ifndef SD_SELF_TEST_ENABLE
#define SD_SELF_TEST_ENABLE SD_SELF_TEST_ENABLE_DEFAULT
#endif

#ifndef SD_BENCH_ENABLE
#define SD_BENCH_ENABLE 0U
#endif

/*
 * SD benchmark profile selector:
 * 0: Custom (use manually defined SD_BENCH_* values)
 * 1: FAST  (quick health check)
 * 2: DEEP  (longer stress run)
 */
#define SD_BENCH_PROFILE_CUSTOM 0U
#define SD_BENCH_PROFILE_FAST   1U
#define SD_BENCH_PROFILE_DEEP   2U

#ifndef SD_BENCH_PROFILE
#define SD_BENCH_PROFILE SD_BENCH_PROFILE_DEEP
#endif

#ifndef SD_BENCH_WRITE_FILE
#define SD_BENCH_WRITE_FILE "SDBENCH.BIN"
#endif

#ifndef SD_BENCH_WRITE_SIZE
#if (SD_BENCH_PROFILE == SD_BENCH_PROFILE_FAST)
#define SD_BENCH_WRITE_SIZE (32U * 1024U)
#elif (SD_BENCH_PROFILE == SD_BENCH_PROFILE_DEEP)
#define SD_BENCH_WRITE_SIZE (1024U * 1024U)
#else
#define SD_BENCH_WRITE_SIZE (64U * 1024U)
#endif
#endif

#ifndef SD_BENCH_WRITE_CHUNK
#if (SD_BENCH_PROFILE == SD_BENCH_PROFILE_DEEP)
#define SD_BENCH_WRITE_CHUNK 4096U
#elif (SD_BENCH_PROFILE == SD_BENCH_PROFILE_FAST)
#define SD_BENCH_WRITE_CHUNK 1024U
#else
#define SD_BENCH_WRITE_CHUNK 1024U
#endif
#endif

#ifndef SD_BENCH_READ_COUNT
#if (SD_BENCH_PROFILE == SD_BENCH_PROFILE_FAST)
#define SD_BENCH_READ_COUNT 32U
#elif (SD_BENCH_PROFILE == SD_BENCH_PROFILE_DEEP)
#define SD_BENCH_READ_COUNT 128U
#else
#define SD_BENCH_READ_COUNT 64U
#endif
#endif

#ifndef SD_BENCH_READ_CHUNK
#if (SD_BENCH_PROFILE == SD_BENCH_PROFILE_DEEP)
#define SD_BENCH_READ_CHUNK 4096U
#elif (SD_BENCH_PROFILE == SD_BENCH_PROFILE_FAST)
#define SD_BENCH_READ_CHUNK 1024U
#else
#define SD_BENCH_READ_CHUNK 1024U
#endif
#endif

#endif
