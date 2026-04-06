# MJPEG Playback Failure Summary (2026-04-06)

## 1) Current Symptoms

- Boot sequence is normal.
- Entering AVI playback triggers repeated decode failures.
- After some time, system may hang/fault (HardFault snapshot previously pointed to QSPI write-enable path).
- After reset, system no longer crashes immediately, but playback still does not succeed.

## 2) Latest Confirmed Evidence (from logs)

Common failing pattern:

- `MJPEG: decode enter len=...`
- `MJPEG: dma aligned len=...`
- `MJPEG: dma start len=...`
- `MJPEG: JPEG err=0x00000000`
- `MJPEG: dma info_cb=1`
- `MJPEG: dma get_cb=0`
- `MJPEG: dma out_cb=1`
- `MJPEG: dma cvt_ready=1`
- `MJPEG: HAL_JPEG_Decode_DMA failed`

Occasional alternate failing pattern:

- `MJPEG: dma info_cb=0`
- `MJPEG: dma get_cb=1`
- `MJPEG: dma out_cb=0`
- `MJPEG: dma cvt_ready=0`
- `MJPEG: dma timeout`
- `MJPEG: dma input exhausted`

Additional runtime issue:

- `LOG: buffer overflow`

## 3) What This Means

### 3.1 Decode callback chain is partially working

- `info_cb=1` and `out_cb=1` prove JPEG header parse and output callback are reached.
- This excludes simple "JPEG IRQ not enabled" class issues.

### 3.2 Failure is likely after first output callback

- `out_cb=1` with immediate decode failure strongly suggests abort in/after first output chunk processing.
- `JPEG err=0x00000000` indicates HAL error code is not capturing the exact software-side failure reason yet.

### 3.3 Some frames also fail earlier on input side

- `info_cb=0 + get_cb=1 + timeout + input exhausted` indicates a second failure mode: not enough parsable input for JPEG core progression (or parser did not reach valid header callback path).

### 3.4 HardFault location is likely a separate/secondary path

- PC `0x0802614D` maps to `QSPI_W25Qxx_WriteEnable` (`qspi_w25q64.c:67`) around `HAL_QSPI_AutoPolling` call setup.
- This does not match the primary MJPEG decode failure logs directly.
- It is likely a different trigger path (or later side effect), not the first decode failure root cause.

## 4) Files Involved

- `Drivers/User/Src/mjpeg_player.c`
- `Core/Src/stm32h7xx_it.c`
- `Drivers/User/Src/qspi_w25q64.c`
- `Core/Src/main.c`

## 5) Most Probable Root Causes (Ranked)

1. Software-side abort condition in MJPEG decode pipeline after first `DataReady` callback, but without explicit reason logging.
2. Secondary input starvation/timeout path on some frames (`get_cb=1`, no info/out callbacks).
3. Logging channel overflow (`LOG: buffer overflow`) hiding critical timing and state transitions.
4. Independent QSPI runtime fault path still present under specific conditions.

## 6) Next Modification Plan (Recommended Order)

### Step A: Add precise decode fail reason flags (highest priority)

In `mjpeg_player.c`, add explicit counters/flags for:

- `error_callback_count` (increment only in `HAL_JPEG_ErrorCallback`)
- `abort_from_flush_count` (increment when `mjpeg_flush_dma_stage` aborts)
- `abort_from_timeout_count`
- `final_out_len_nonzero_count`
- `convert_block_index` and `convert_total_mcus` at failure time

Goal: distinguish HAL-side DMA/JPEG error vs software abort vs incomplete final block.

### Step B: Handle trailing partial stage bytes safely

In `mjpeg_decode_frame_via_dma`, re-check final logic:

- current behavior treats non-zero trailing `out_len` as hard error.
- temporarily allow dropping tiny trailing bytes after decode complete (for diagnosis) and log exact value.

Goal: verify whether strict tail handling is causing false-negative decode failures.

### Step C: Reduce work inside output callback

Current callback path may do heavy processing.

- keep callback minimal (copy + requeue output buffer),
- defer expensive color conversion work to non-interrupt decode loop context with safe synchronization.

Goal: avoid callback-latency-induced pipeline issues.

### Step D: Separate QSPI fault from MJPEG debug run

During MJPEG debug runs:

- ensure QSPI write/erase/download paths are not entered,
- keep only read/init path if needed,
- confirm whether HardFault disappears entirely.

Goal: isolate primary decoder bug from unrelated flash write path.

### Step E: Throttle logs to prevent channel overflow

- tighten fail log frequency,
- avoid per-frame repeated long prints.

Goal: prevent `LOG: buffer overflow` from obscuring root-cause signals.

## 7) Fast Validation Checklist for Next Run

1. Build and flash.
2. Play same AVI file.
3. Capture only first 10 failing frames.
4. Confirm which reason flag increments first.
5. If decode completes once, verify frame blit path and output dimensions.

## 8) Current Status Snapshot

- Build is successful.
- Playback is still failing.
- Root cause is narrowed to post-callback decode path plus occasional input-timeout path.
- QSPI fault evidence exists but appears orthogonal to the primary playback failure.
