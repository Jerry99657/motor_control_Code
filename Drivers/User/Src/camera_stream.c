#include "camera_stream.h"

#include "camera_service.h"
#include "comm_service.h"
#include "command_control.h"
#include "main.h"
#include "mjpeg_player.h"

#include <stddef.h>
#include <string.h>

#define CAMERA_STREAM_MAGIC_0              0x43U /* C */
#define CAMERA_STREAM_MAGIC_1              0x41U /* A */
#define CAMERA_STREAM_MAGIC_2              0x4DU /* M */
#define CAMERA_STREAM_MAGIC_3              0x31U /* 1 */
#define CAMERA_STREAM_VERSION              0x01U
#define CAMERA_STREAM_FLAG_START           0x01U
#define CAMERA_STREAM_FLAG_END             0x02U
#define CAMERA_STREAM_HEADER_SIZE          22U
#define CAMERA_STREAM_CRC_SIZE              4U
#define CAMERA_STREAM_CHUNK_PAYLOAD      4096U
#define CAMERA_STREAM_PACKET_CAPACITY     \
    (CAMERA_STREAM_HEADER_SIZE + CAMERA_STREAM_CHUNK_PAYLOAD + \
     CAMERA_STREAM_CRC_SIZE)
#define CAMERA_STREAM_REQUEST_TIMEOUT_MS  2000U
#define CAMERA_STREAM_ACK_TIMEOUT_MS       500U
#define CAMERA_STREAM_RETRY_MS            1000U
/* MJPEG_Player_NormalizeJpeg() pads the hardware-decoder input to a 32-byte
 * boundary. Padding is not part of the JPEG file and must not be transported
 * to a browser after the FFD9 end marker. */
#define CAMERA_STREAM_MAX_JPEG_PADDING      31U

static uint8_t s_packet[CAMERA_STREAM_PACKET_CAPACITY]
    __attribute__((aligned(32)));
static const uint8_t *s_frame;
static uint32_t s_frame_size;
static uint32_t s_frame_offset;
static uint16_t s_frame_sequence;
static uint16_t s_chunk_payload_size;
static uint16_t s_last_ack_sequence;
static uint32_t s_remote_request_tick;
static uint32_t s_ack_wait_started_tick;
static uint32_t s_retry_tick;
static uint8_t s_remote_requested;
static uint8_t s_camera_owned;
static uint8_t s_chunk_in_flight;
static uint8_t s_waiting_ack;
static uint8_t s_ack_received;
static CameraStreamStats s_stats;

static void camera_stream_write_u16(uint8_t *destination, uint16_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
}

static void camera_stream_write_u32(uint8_t *destination, uint32_t value)
{
    destination[0] = (uint8_t)(value & 0xFFU);
    destination[1] = (uint8_t)((value >> 8U) & 0xFFU);
    destination[2] = (uint8_t)((value >> 16U) & 0xFFU);
    destination[3] = (uint8_t)((value >> 24U) & 0xFFU);
}

static uint32_t camera_stream_crc32_update(uint32_t crc,
                                           const uint8_t *data,
                                           uint32_t length)
{
    uint32_t index;

    for (index = 0U; index < length; ++index)
    {
        uint8_t bit;

        crc ^= data[index];
        for (bit = 0U; bit < 8U; ++bit)
        {
            crc = ((crc & 1U) != 0U) ?
                  ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
        }
    }
    return crc;
}

static void camera_stream_release_frame(void)
{
    if (s_frame != NULL)
    {
        Camera_Service_ReleaseSnapshot(s_frame);
    }
    s_frame = NULL;
    s_frame_size = 0U;
    s_frame_offset = 0U;
    s_chunk_payload_size = 0U;
}

static void camera_stream_stop_camera(void)
{
    camera_stream_release_frame();
    if (s_camera_owned != 0U)
    {
        Camera_Service_Sleep();
    }
    s_camera_owned = 0U;
    s_chunk_in_flight = 0U;
    s_waiting_ack = 0U;
    s_ack_received = 0U;
}

static uint8_t camera_stream_request_is_live(uint32_t now)
{
    if ((s_remote_requested == 0U) ||
        ((now - s_remote_request_tick) >= CAMERA_STREAM_REQUEST_TIMEOUT_MS))
    {
        s_remote_requested = 0U;
        return 0U;
    }

    return (CommandControl_IsActive() != 0U) ? 1U : 0U;
}

static uint16_t camera_stream_build_packet(void)
{
    uint32_t remaining = s_frame_size - s_frame_offset;
    uint32_t crc;
    uint8_t flags = 0U;
    uint16_t packet_length;

    s_chunk_payload_size = (remaining > CAMERA_STREAM_CHUNK_PAYLOAD) ?
                           CAMERA_STREAM_CHUNK_PAYLOAD :
                           (uint16_t)remaining;
    if (s_frame_offset == 0U)
    {
        flags |= CAMERA_STREAM_FLAG_START;
    }
    if ((s_frame_offset + s_chunk_payload_size) >= s_frame_size)
    {
        flags |= CAMERA_STREAM_FLAG_END;
    }

    s_packet[0] = CAMERA_STREAM_MAGIC_0;
    s_packet[1] = CAMERA_STREAM_MAGIC_1;
    s_packet[2] = CAMERA_STREAM_MAGIC_2;
    s_packet[3] = CAMERA_STREAM_MAGIC_3;
    s_packet[4] = CAMERA_STREAM_VERSION;
    s_packet[5] = flags;
    camera_stream_write_u16(&s_packet[6], s_frame_sequence);
    camera_stream_write_u32(&s_packet[8], s_frame_offset);
    camera_stream_write_u32(&s_packet[12], s_frame_size);
    camera_stream_write_u16(&s_packet[16], CAMERA_JPEG_WIDTH);
    camera_stream_write_u16(&s_packet[18], CAMERA_JPEG_HEIGHT);
    camera_stream_write_u16(&s_packet[20], s_chunk_payload_size);
    memcpy(&s_packet[CAMERA_STREAM_HEADER_SIZE],
           &s_frame[s_frame_offset], s_chunk_payload_size);

    packet_length = (uint16_t)(CAMERA_STREAM_HEADER_SIZE +
                               s_chunk_payload_size +
                               CAMERA_STREAM_CRC_SIZE);
    crc = camera_stream_crc32_update(
        0xFFFFFFFFUL, &s_packet[4],
        (uint32_t)(packet_length - 4U - CAMERA_STREAM_CRC_SIZE));
    crc ^= 0xFFFFFFFFUL;
    camera_stream_write_u32(&s_packet[packet_length - CAMERA_STREAM_CRC_SIZE],
                            crc);
    return packet_length;
}

static void camera_stream_drop_current_frame(void)
{
    camera_stream_release_frame();
    s_stats.frames_dropped++;
    s_waiting_ack = 0U;
    s_ack_received = 0U;
}

static uint32_t camera_stream_transport_jpeg_size(const uint8_t *frame,
                                                   uint32_t padded_size)
{
    uint32_t candidate = padded_size;
    uint32_t trailing_bytes = 0U;

    if ((frame == NULL) || (padded_size < 4U))
    {
        return 0U;
    }

    while ((candidate >= 2U) &&
           (trailing_bytes <= CAMERA_STREAM_MAX_JPEG_PADDING))
    {
        if ((frame[candidate - 2U] == 0xFFU) &&
            (frame[candidate - 1U] == 0xD9U))
        {
            return candidate;
        }
        candidate--;
        trailing_bytes++;
    }

    return 0U;
}

void CameraStream_Init(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
    s_frame = NULL;
    s_frame_size = 0U;
    s_frame_offset = 0U;
    s_frame_sequence = 0U;
    s_chunk_payload_size = 0U;
    s_last_ack_sequence = 0U;
    s_remote_request_tick = 0U;
    s_ack_wait_started_tick = 0U;
    s_retry_tick = 0U;
    s_remote_requested = 0U;
    s_camera_owned = 0U;
    s_chunk_in_flight = 0U;
    s_waiting_ack = 0U;
    s_ack_received = 0U;
}

void CameraStream_SetRemoteEnabled(uint8_t enabled)
{
    if (enabled != 0U)
    {
        s_remote_requested = 1U;
        s_remote_request_tick = HAL_GetTick();
    }
    else
    {
        s_remote_requested = 0U;
    }
}

void CameraStream_AcknowledgeFrame(uint16_t sequence)
{
    s_last_ack_sequence = sequence;
    s_ack_received = 1U;
}

void CameraStream_Process(void)
{
    Camera_Result camera_result;
    Camera_State camera_state;
    uint32_t now = HAL_GetTick();

    if (camera_stream_request_is_live(now) == 0U)
    {
        camera_stream_stop_camera();
        s_stats.requested = s_remote_requested;
        s_stats.camera_owned = 0U;
        return;
    }

    s_stats.requested = 1U;
    if (s_camera_owned == 0U)
    {
        if ((int32_t)(now - s_retry_tick) < 0)
        {
            return;
        }

        /* Command Control and local media/camera pages are mutually
         * exclusive. Start from a known OFF state after a previous error. */
        if (Camera_Service_GetState() != CAMERA_STATE_OFF)
        {
            Camera_Service_Sleep();
        }
        Camera_Service_SetJpegProfile(CAMERA_JPEG_PROFILE_WEB_STREAM);
        camera_result = Camera_Service_Init();
        if (camera_result != CAMERA_RESULT_OK)
        {
            Camera_Service_Sleep();
            s_stats.camera_error_count++;
            s_retry_tick = now + CAMERA_STREAM_RETRY_MS;
            return;
        }
        s_camera_owned = 1U;
        s_stats.camera_owned = 1U;
    }

    if (s_chunk_in_flight != 0U)
    {
        if (CommService_UartStreamBusy() != 0U)
        {
            return;
        }

        s_chunk_in_flight = 0U;
        s_frame_offset += s_chunk_payload_size;
        if (s_frame_offset >= s_frame_size)
        {
            camera_stream_release_frame();
            s_stats.frames_completed++;
            s_waiting_ack = 1U;
            s_ack_wait_started_tick = now;
        }
    }

    if (s_waiting_ack != 0U)
    {
        if ((s_ack_received != 0U) &&
            (s_last_ack_sequence == s_frame_sequence))
        {
            s_waiting_ack = 0U;
            s_ack_received = 0U;
        }
        else if ((now - s_ack_wait_started_tick) >=
                 CAMERA_STREAM_ACK_TIMEOUT_MS)
        {
            s_waiting_ack = 0U;
            s_ack_received = 0U;
            s_stats.ack_timeout_count++;
        }
        else
        {
            return;
        }
    }

    camera_state = Camera_Service_GetState();
    if (camera_state == CAMERA_STATE_ERROR)
    {
        camera_stream_drop_current_frame();
        Camera_Service_Sleep();
        s_camera_owned = 0U;
        s_stats.camera_owned = 0U;
        s_stats.camera_error_count++;
        s_retry_tick = now + CAMERA_STREAM_RETRY_MS;
        return;
    }

    if (s_frame == NULL)
    {
        if (camera_state == CAMERA_STATE_FRAME_READY)
        {
            camera_result = Camera_Service_GetSnapshot(&s_frame,
                                                       &s_frame_size);
            if (camera_result != CAMERA_RESULT_OK)
            {
                s_stats.camera_error_count++;
                return;
            }

            if (MJPEG_Player_NormalizeJpeg((uint8_t *)s_frame,
                                           &s_frame_size,
                                           CAMERA_JPEG_BUFFER_CAPACITY) !=
                MJPEG_PLAYER_OK)
            {
                camera_stream_drop_current_frame();
                return;
            }

            s_frame_size = camera_stream_transport_jpeg_size(s_frame,
                                                              s_frame_size);
            if (s_frame_size == 0U)
            {
                camera_stream_drop_current_frame();
                return;
            }

            s_frame_offset = 0U;
            s_frame_sequence++;
            s_ack_received = 0U;
            s_stats.frames_started++;
            s_stats.last_frame_size = s_frame_size;
            s_stats.last_frame_sequence = s_frame_sequence;

            /* Fill slot N+1 while slot N is being packetized and sent. */
            camera_result = Camera_Service_StartSnapshot(
                CAMERA_CAPTURE_TIMEOUT_DEFAULT);
            if ((camera_result != CAMERA_RESULT_OK) &&
                (camera_result != CAMERA_RESULT_BUSY))
            {
                camera_stream_drop_current_frame();
                s_stats.camera_error_count++;
                return;
            }
        }
        else if (camera_state == CAMERA_STATE_READY)
        {
            camera_result = Camera_Service_StartSnapshot(
                CAMERA_CAPTURE_TIMEOUT_DEFAULT);
            if ((camera_result != CAMERA_RESULT_OK) &&
                (camera_result != CAMERA_RESULT_BUSY))
            {
                s_stats.camera_error_count++;
            }
            return;
        }
        else
        {
            return;
        }
    }

    if ((s_frame != NULL) && (s_chunk_in_flight == 0U))
    {
        uint16_t packet_length = camera_stream_build_packet();

        if (CommService_UartStreamSend(s_packet, packet_length) != 0U)
        {
            s_chunk_in_flight = 1U;
            s_stats.packets_sent++;
        }
        else
        {
            s_stats.uart_busy_count++;
        }
    }
}

void CameraStream_GetStats(CameraStreamStats *stats)
{
    if (stats != NULL)
    {
        s_stats.requested = s_remote_requested;
        s_stats.camera_owned = s_camera_owned;
        *stats = s_stats;
    }
}

uint8_t CameraStream_NeedsFastService(void)
{
    return ((s_remote_requested != 0U) ||
            (s_camera_owned != 0U) ||
            (s_chunk_in_flight != 0U) ||
            (s_waiting_ack != 0U)) ? 1U : 0U;
}
