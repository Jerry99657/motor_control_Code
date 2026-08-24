#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <WebSocketsServer.h>
#include <ESPmDNS.h>
#include <esp_system.h>
#include <esp_wifi.h>
#include <esp_rom_crc.h>
#include <errno.h>
#include <lwip/sockets.h>

#include "flydigi_gamepad.h"

// Prefer the shared router; keep the original SoftAP as a recovery path.
const char* staSsid = "HUAWEI-A48L7A_HiLink";
const char* staPassword = "88888888@";
const char* fallbackApSsid = "ESP32_JoyStick";
const char* fallbackApPassword = "12345678"; // At least 8 characters
const char* mdnsHostname = "esp32-controller";

class CameraWebSocketsServer : public WebSocketsServer {
 public:
  using WebSocketsServer::WebSocketsServer;

  bool enableLowLatency(uint8_t clientNumber) {
    if(clientNumber >= WEBSOCKETS_SERVER_CLIENT_MAX) return false;
    WSclient_t* client = &_clients[clientNumber];
    if(!clientIsConnected(client) || (client->tcp == nullptr)) return false;
    return (client->tcp->setNoDelay(true) == 0) &&
           client->tcp->getNoDelay();
  }

  int sendRawNonBlocking(uint8_t clientNumber, const uint8_t* data,
                         size_t length) {
    if((clientNumber >= WEBSOCKETS_SERVER_CLIENT_MAX) ||
       (data == nullptr) || (length == 0U)) return -1;

    WSclient_t* client = &_clients[clientNumber];
    if(!clientIsConnected(client) || (client->tcp == nullptr)) return -1;

    const int socketFd = client->tcp->fd();
    if(socketFd < 0) return -1;

    const int sent = ::send(socketFd, data, length, MSG_DONTWAIT);
    if((sent < 0) && ((errno == EAGAIN) || (errno == EWOULDBLOCK))) return 0;
    return sent;
  }

  void dropClientNow(uint8_t clientNumber) {
    if(clientNumber >= WEBSOCKETS_SERVER_CLIENT_MAX) return;
    WSclient_t* client = &_clients[clientNumber];
    if(clientIsConnected(client)) clientDisconnect(client);
  }
};

WebServer server(80);
WebSocketsServer webSocket(81);
CameraWebSocketsServer cameraWebSocket(82);
DNSServer dnsServer;
const byte DNS_PORT = 53;

unsigned long lastSendTime = 0;
const int sendIntervalMs = 20; // 50 Hz UART update rate
/* Mobile browsers can briefly defer JavaScript timers while processing touch
 * and layout work. Keep a live-control watchdog, but tolerate short stalls. */
const uint32_t JOYSTICK_TIMEOUT_MS = 1000;
const uint32_t NES_INPUT_TIMEOUT_MS = 350;
const uint32_t NES_KEEPALIVE_MS = 50;
const uint32_t AP_RETRY_INTERVAL_MS = 2000;
const uint32_t STA_CONNECT_TIMEOUT_MS = 12000;
const uint32_t STA_RETRY_INTERVAL_MS = 5000;
const uint32_t STA_FALLBACK_DELAY_MS = 12000;
const uint32_t MDNS_RETRY_INTERVAL_MS = 10000;
const uint32_t STATUS_INTERVAL_MS = 5000;
const uint32_t AP_HEALTH_INTERVAL_MS = 3000;
const uint32_t CAMERA_REQUEST_INTERVAL_MS = 500;
/* Camera turbo is enabled only after NimBLE has initialized safely.  With
  * idle BLE discovery paused, complete each JPEG in a short burst rather than
  * keeping a low-rate TCP transfer queued for seconds. Wi-Fi remains in
  * MIN_MODEM power save so the board can run from the external 5 V rail. The
  * small control guard still gives port 81 ACKs first service. */
/* Browser credit, cameraWebTxActive and the deferred STM32 frame ACK now form
 * the complete backpressure chain. An additional time gate would discard the
 * frame released immediately by that ACK, so no fixed gap is needed. */
const uint32_t CAMERA_WEB_FRAME_INTERVAL_MS = 0;
const uint32_t CAMERA_WEB_CHUNK_INTERVAL_MS = 0;
const uint32_t CAMERA_WEB_TX_CHUNK_BYTES = 4096;
const uint32_t CAMERA_CONTROL_ACTIVE_GUARD_MS = 8;
const uint32_t CAMERA_CONTROL_IDLE_GUARD_MS = 3;
const uint32_t UART_LINK_BAUD = 2500000;
const int8_t WIFI_TX_POWER_QDBM = 60; // 15 dBm, API unit is 0.25 dBm

uint32_t lastJoystickUpdateMs = 0;
uint32_t lastApRetryMs = 0;
uint32_t lastStatusMs = 0;
uint32_t lastApHealthMs = 0;
bool joystickActive = false;
bool gyroEnabled = false;
bool gyroCommandDirty = true;
int8_t gyroSignedSpeed = 0;
enum ControlMode : uint8_t {
  CONTROL_MODE_MECANUM = 0,
  CONTROL_MODE_NES = 1
};
enum InputSource : uint8_t {
  INPUT_SOURCE_NONE = 0,
  INPUT_SOURCE_WEB = 1,
  INPUT_SOURCE_GAMEPAD = 2
};
ControlMode controlMode = CONTROL_MODE_MECANUM;
InputSource activeInputSource = INPUT_SOURCE_NONE;
uint8_t nesButtons = 0;
uint8_t nesSequence = 0;
uint8_t gamepadNesSequence = 0;
uint32_t lastNesUpdateMs = 0;
uint32_t lastNesSendMs = 0;
bool nesCommandDirty = true;
uint8_t nesResetSequence = 0;
bool nesResetCommandPending = false;
volatile bool apRunning = false;
volatile bool staGotIp = false;
volatile bool staDisconnected = false;
bool staConnected = false;
bool mdnsRunning = false;
bool gamepadBleEnabled = false;
uint8_t selectedApChannel = 6;
uint8_t apHealthFailures = 0;
uint8_t activeWebSocketClient = 0xFF;
uint8_t activeCameraWebSocketClient = 0xFF;
uint32_t activeWebSocketSession = 0;
uint32_t revokedWebSocketSessions[4] = {0, 0, 0, 0};
uint8_t revokedWebSocketWriteIndex = 0;
uint32_t joystickRxCount = 0;
uint32_t joystickRejectedCount = 0;
uint32_t joystickTakeoverCount = 0;
uint16_t lastJoystickSequence = 0;
bool joystickSequenceValid = false;
volatile bool apClientDisconnected = false;
volatile bool apStopped = false;
uint32_t lastStaRetryMs = 0;
uint32_t staDisconnectedSinceMs = 0;
uint32_t lastMdnsRetryMs = 0;
uint32_t lastCameraRequestMs = 0;
bool cameraStreamRequested = false;
bool wifiCameraTurbo = false;
uint32_t cameraTurboLastWantedMs = 0U;
uint32_t cameraTurboLastAttemptMs = 0U;

bool setWifiHighPerformanceMode(bool enabled);

const uint8_t CAMERA_UART_DEVICE_ID = 0x0F;
const uint8_t CAMERA_UART_CMD_ENABLE = 0x02;
const uint8_t CAMERA_UART_CMD_FRAME_ACK = 0x03;
const uint8_t CAMERA_MAGIC[4] = {0x43, 0x41, 0x4D, 0x31}; // "CAM1"
const uint16_t CAMERA_HEADER_SIZE = 22;
const uint16_t CAMERA_CRC_SIZE = 4;
const uint16_t CAMERA_MAX_PAYLOAD = 4096;
const uint32_t CAMERA_MAX_FRAME_SIZE = 64U * 1024U;
const uint16_t CAMERA_UART_READ_CHUNK = 4096U;
const uint16_t CAMERA_UART_READ_BUDGET = 8192U;
const uint16_t CAMERA_PACKET_CAPACITY =
    CAMERA_HEADER_SIZE + CAMERA_MAX_PAYLOAD + CAMERA_CRC_SIZE;

enum CameraUartRxState : uint8_t {
  CAMERA_UART_FIND_MAGIC = 0,
  CAMERA_UART_READ_HEADER,
  CAMERA_UART_READ_PACKET
};

CameraUartRxState cameraUartRxState = CAMERA_UART_FIND_MAGIC;
uint8_t cameraUartPacket[CAMERA_PACKET_CAPACITY];
uint16_t cameraUartPacketIndex = 0;
uint16_t cameraUartExpectedLength = 0;
uint8_t cameraUartMagicMatched = 0;
uint32_t cameraUartPacketCount = 0;
uint32_t cameraUartCrcErrorCount = 0;
uint32_t cameraUartFormatErrorCount = 0;
uint32_t cameraUartBytesRead = 0;
uint32_t cameraUartBulkReadCount = 0;
uint32_t cameraUartMaxBacklog = 0;
volatile uint32_t cameraUartFifoOverflowCount = 0;
volatile uint32_t cameraUartBufferFullCount = 0;
volatile uint32_t cameraUartFrameErrorCount = 0;
volatile uint32_t cameraUartParityErrorCount = 0;
volatile uint32_t cameraUartBreakErrorCount = 0;
volatile uint8_t cameraUartLastDriverError = UART_NO_ERROR;
uint32_t cameraWebSocketDropCount = 0;
uint32_t cameraLastPacketMs = 0;
uint8_t cameraJpegFrame[CAMERA_MAX_FRAME_SIZE];
uint32_t cameraJpegExpectedSize = 0;
uint32_t cameraJpegOffset = 0;
uint16_t cameraJpegSequence = 0;
bool cameraJpegAssembling = false;
uint32_t cameraJpegCompletedCount = 0;
uint32_t cameraJpegAssemblyDropCount = 0;
uint32_t cameraJpegStartCount = 0;
uint32_t cameraJpegStartOverlapCount = 0;
uint32_t cameraJpegStartTxSuppressedCount = 0;
uint32_t cameraJpegStartRateSuppressedCount = 0;
uint32_t cameraJpegOrderDropCount = 0;
uint32_t cameraJpegMarkerDropCount = 0;
uint16_t cameraJpegLastDropSequence = 0;
uint32_t cameraJpegLastExpectedOffset = 0;
uint32_t cameraJpegLastReceivedOffset = 0;
uint32_t cameraJpegWebSentCount = 0;
uint32_t cameraJpegThrottleDropCount = 0;
uint32_t cameraLastWebFrameMs = 0;
bool cameraWebTxActive = false;
uint8_t cameraWebTxClient = 0xFFU;
uint8_t cameraWebTxHeader[10];
uint8_t cameraWebTxHeaderSize = 0U;
uint8_t cameraWebTxHeaderOffset = 0U;
uint32_t cameraWebTxSize = 0U;
uint32_t cameraWebTxOffset = 0U;
uint32_t cameraWebTxLastProgressMs = 0U;
uint32_t cameraWebTxLastChunkMs = 0U;
uint32_t cameraWebTxStartedMs = 0U;
uint32_t cameraWebTxLastDurationMs = 0U;
uint32_t cameraWebTxMaxDurationMs = 0U;
uint32_t cameraWebTxWouldBlockCount = 0U;
uint32_t cameraWebTxPartialWriteCount = 0U;
bool cameraFrameAckPending = false;
uint16_t cameraFrameAckSequence = 0U;
uint32_t cameraFrameAckStartedMs = 0U;
uint32_t cameraFrameAckDeferredCount = 0U;
uint32_t cameraFrameAckReleasedCount = 0U;
uint32_t cameraControlGuardUntilMs = 0U;
bool cameraBrowserCredit = false;
uint8_t cameraUartReadBuffer[CAMERA_UART_READ_CHUNK];
bool cameraRomCrcValid = true;

// Joystick values (-100 ~ 100)
int lx = 0, ly = 0, rx = 0, ry = 0;

// Protocol parameters
const uint8_t HEADER_1 = 0x77;
const uint8_t HEADER_2 = 0x68;
const uint8_t FRAME_END = 0x0A;
const uint8_t WS_TYPE_MODE = 0x4D; // 'M'
const uint8_t WS_TYPE_NES = 0x4E;  // 'N'
const uint8_t WS_TYPE_NES_RESET = 0x52; // 'R'
const uint8_t WS_TYPE_ACK = 0x41;   // 'A'
const uint8_t WS_TYPE_HELLO = 0x48; // 'H'
const uint8_t WS_TYPE_AXES = 0x4A;  // 'J'
const uint8_t WS_TYPE_GYRO = 0x47;  // 'G'
const uint8_t UART_DEV_NES = 0x0E;

// HTML/JS Frontend - Highly visible, Captive Portal compatible
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1.0, user-scalable=no, maximum-scale=1.0">
  <title>ESP32 Xbox Web Controller</title>
  <style>
    :root { --gyro-color:#4b5563; --gyro-glow:rgba(75,85,99,.35); --nes-red:#7b1822; --nes-red-dark:#3f0c13; --nes-cream:#e8dfc8; }
    body { margin:0; padding:0; min-height:100vh; background:radial-gradient(circle at 50% 0%,#1f2937 0%,#111 52%,#090b10 100%); color:#fff; overflow:hidden; touch-action:none; font-family:Arial,sans-serif; }
    .title { text-align: center; margin-top: 10px; font-size: 16px; font-weight: bold; pointer-events: none; color: #00ffcc;}
    .mode-switch { position:fixed; top:8px; right:10px; z-index:30; min-width:92px; height:32px; padding:0 13px; border:1px solid rgba(255,255,255,.2); border-radius:16px; color:#08111d; background:linear-gradient(135deg,#67e8f9,#22d3ee); box-shadow:0 5px 18px rgba(0,0,0,.35); font-size:11px; font-weight:900; letter-spacing:.5px; touch-action:manipulation; transition:transform .14s cubic-bezier(.2,1.7,.4,1),background .2s,color .2s; }
    .mode-switch:active { transform:scale(.9); }
    .mode-switch.nes { color:#f8ead0; background:linear-gradient(145deg,#9a2632,#5b1019); }
    .input-switch { position:fixed; top:8px; left:10px; z-index:30; min-width:112px; max-width:190px; height:32px; padding:0 13px; overflow:hidden; border:1px solid rgba(255,255,255,.2); border-radius:16px; color:#dbeafe; background:linear-gradient(145deg,#334155,#1e293b); box-shadow:0 5px 18px rgba(0,0,0,.35); font-size:10px; font-weight:900; letter-spacing:.45px; text-overflow:ellipsis; white-space:nowrap; touch-action:manipulation; transition:transform .14s cubic-bezier(.2,1.7,.4,1),background .2s,color .2s,box-shadow .2s; }
    .input-switch:active { transform:scale(.92); }
    .input-switch.pc { color:#082f49; background:linear-gradient(135deg,#bae6fd,#38bdf8); }
    .input-switch.ready { color:#052e16; background:linear-gradient(135deg,#86efac,#22c55e); box-shadow:0 5px 18px rgba(0,0,0,.35),0 0 18px rgba(34,197,94,.25); }
    .input-switch.blocked { color:#450a0a; background:linear-gradient(135deg,#fecaca,#f87171); }
    .mode-page[hidden] { display:none !important; }
    #debug { position: absolute; top: 35px; width: 100%; text-align: center; font-size: 14px; color: #fff; pointer-events: none; text-shadow: 0px 0px 5px #000; z-index: 10;}
    .gyro-panel { box-sizing:border-box; width:min(620px,calc(100% - 28px)); height:76px; margin:42px auto 0; padding:10px 14px; display:grid; grid-template-columns:104px 1fr; gap:16px; align-items:center; border:1px solid rgba(255,255,255,.12); border-radius:18px; background:linear-gradient(145deg,rgba(36,45,61,.94),rgba(18,23,32,.94)); box-shadow:0 12px 30px rgba(0,0,0,.34),inset 0 1px 0 rgba(255,255,255,.08); }
    #gyroToggle { height:46px; border:1px solid rgba(255,255,255,.16); border-radius:14px; background:linear-gradient(180deg,#374151,#202734); color:#d1d5db; font-size:13px; font-weight:800; letter-spacing:.7px; touch-action:manipulation; transition:transform .16s cubic-bezier(.2,1.7,.4,1),box-shadow .2s,background .2s,color .2s; }
    #gyroToggle:active { transform:scale(.92); }
    #gyroToggle.on { color:#08111d; background:linear-gradient(135deg,#67e8f9,#22d3ee 48%,#60a5fa); box-shadow:0 0 22px rgba(34,211,238,.42),inset 0 1px 0 rgba(255,255,255,.65); transform:scale(1.035); }
    .gyro-slider-wrap { min-width:0; }
    .gyro-readout { display:flex; align-items:center; justify-content:space-between; margin:0 2px 7px; color:#cbd5e1; font-size:12px; font-weight:700; }
    #gyroValue { min-width:96px; text-align:right; color:var(--gyro-color); text-shadow:0 0 12px var(--gyro-glow); transition:color .15s,text-shadow .15s,transform .15s cubic-bezier(.2,1.7,.4,1); }
    #gyroValue.pulse { transform:scale(1.12); }
    #gyroSlider { appearance:none; -webkit-appearance:none; width:100%; height:12px; margin:0; border-radius:999px; outline:none; background:linear-gradient(90deg,#f59e0b 0%,#2a303b 50%,#2a303b 100%); box-shadow:inset 0 2px 5px rgba(0,0,0,.5),0 0 14px var(--gyro-glow); touch-action:pan-x; }
    #gyroSlider::-webkit-slider-thumb { -webkit-appearance:none; width:28px; height:28px; border-radius:50%; border:3px solid #f8fafc; background:var(--gyro-color); box-shadow:0 0 0 5px rgba(255,255,255,.08),0 0 18px var(--gyro-glow); transition:transform .12s cubic-bezier(.2,1.8,.4,1),background .15s; }
    #gyroSlider:active::-webkit-slider-thumb { transform:scale(1.2); }
    #gyroSlider::-moz-range-thumb { width:22px; height:22px; border-radius:50%; border:3px solid #f8fafc; background:var(--gyro-color); box-shadow:0 0 18px var(--gyro-glow); }
    .gyro-scale { display:flex; justify-content:space-between; margin-top:5px; color:#64748b; font-size:10px; font-weight:700; letter-spacing:.4px; }
    .container { display:flex; height:calc(100vh - 145px); min-height:190px; width:100%; align-items:center; justify-content:space-around; gap:10px; pointer-events:none; transform:translateY(8px); }
    .joy-wrap { text-align: center; pointer-events: auto; }
    .camera-panel { position:relative; box-sizing:border-box; flex:0 1 288px; width:min(288px,34vw); aspect-ratio:4/3; overflow:hidden; border:1px solid rgba(103,232,249,.42); border-radius:18px; background:linear-gradient(145deg,#111827,#05080d); box-shadow:0 12px 30px rgba(0,0,0,.42),inset 0 0 0 1px rgba(255,255,255,.05),0 0 22px rgba(34,211,238,.12); pointer-events:none; }
    #cameraImage { display:block; width:100%; height:100%; object-fit:contain; opacity:0; transition:opacity .18s ease; }
    #cameraImage.ready { opacity:1; }
    #cameraPlaceholder { position:absolute; inset:0; display:flex; align-items:center; justify-content:center; padding:18px; color:#94a3b8; text-align:center; font-size:12px; font-weight:800; line-height:1.5; letter-spacing:.5px; }
    .camera-badge { position:absolute; left:8px; right:8px; bottom:7px; display:flex; justify-content:space-between; gap:8px; padding:5px 8px; border-radius:9px; color:#dff9ff; background:rgba(3,10,18,.72); backdrop-filter:blur(4px); font-size:9px; font-weight:800; letter-spacing:.4px; }
    #cameraState.live { color:#67e8f9; }
    /* Making Canvas Highly Visible */
    canvas { 
      background: radial-gradient(circle, #444 0%, #222 100%); 
      border-radius: 50%; 
      border: 3px solid #555;
      box-shadow: 0 0 20px rgba(0,0,0,0.8) inset, 0 0 10px rgba(255,255,255,0.1); 
      touch-action: none; 
    }
    .label { margin-top: 10px; color: #aaa; font-size: 14px; pointer-events: none; font-weight: bold;}
    /* PC gamepad layout: the enlarged camera is the top of the triangle and
       the two read-only stick mirrors form its lower corners. */
    #robotPage.pc-gamepad .container { box-sizing:border-box; width:min(940px,100%); margin:0 auto; padding:6px 28px 12px; display:grid; grid-template-columns:1fr 1fr; grid-template-rows:minmax(190px,1fr) 126px; column-gap:80px; row-gap:2px; align-items:center; justify-items:center; transform:none; }
    #robotPage.pc-gamepad .camera-panel { grid-column:1 / 3; grid-row:1; width:min(520px,62vw); border-color:rgba(103,232,249,.72); box-shadow:0 16px 42px rgba(0,0,0,.48),inset 0 0 0 1px rgba(255,255,255,.07),0 0 28px rgba(34,211,238,.2); }
    #robotPage.pc-gamepad .joy-wrap:first-child { grid-column:1; grid-row:2; }
    #robotPage.pc-gamepad .joy-wrap:last-child { grid-column:2; grid-row:2; }
    #robotPage.pc-gamepad .joy-wrap { pointer-events:none; opacity:.9; }
    #robotPage.pc-gamepad canvas { width:108px; height:108px; border-width:2px; box-shadow:0 0 14px rgba(0,0,0,.8) inset,0 0 12px rgba(103,232,249,.1); }
    #robotPage.pc-gamepad .label { margin-top:3px; font-size:11px; letter-spacing:.4px; }
    .nes-page { box-sizing:border-box; width:100%; height:calc(100vh - 34px); padding:7px 12px 10px; display:flex; align-items:center; justify-content:center; }
    .nes-shell { position:relative; box-sizing:border-box; width:min(920px,96vw); height:min(390px,82vh); min-height:230px; padding:24px 30px; border:5px solid #4a0c13; border-radius:44px; background:linear-gradient(155deg,#9c2e39 0%,#711622 55%,#4b0d15 100%); box-shadow:0 20px 45px rgba(0,0,0,.58),inset 0 2px 0 rgba(255,255,255,.17),inset 0 -8px 18px rgba(0,0,0,.25); }
    .nes-face { box-sizing:border-box; width:100%; height:100%; display:grid; grid-template-columns:1.05fr .8fr 1.05fr; align-items:center; gap:20px; padding:18px 22px; border-radius:30px; color:#1d1b18; background:linear-gradient(160deg,#f2ead4,#d7cdb4); box-shadow:inset 0 0 0 3px rgba(61,30,25,.17),inset 0 -7px 15px rgba(65,42,32,.12); }
    .nes-brand { position:absolute; top:31px; left:50%; transform:translateX(-50%); z-index:2; color:rgba(89,18,28,.72); font-size:11px; font-weight:900; letter-spacing:2.2px; pointer-events:none; }
    .dpad { position:relative; width:174px; height:174px; margin:auto; filter:drop-shadow(0 6px 5px rgba(0,0,0,.35)); }
    .nes-btn { -webkit-tap-highlight-color:transparent; user-select:none; -webkit-user-select:none; touch-action:none; border:0; outline:0; font-family:Arial,sans-serif; font-weight:900; }
    .dpad-btn { position:absolute; width:62px; height:62px; border:3px solid #242424; border-radius:9px; color:#777; background:linear-gradient(145deg,#373737,#111); box-shadow:inset 3px 3px 4px rgba(255,255,255,.08),inset -4px -4px 5px rgba(0,0,0,.55); transition:transform .08s,filter .08s; }
    .dpad-btn.up { left:56px; top:0; }
    .dpad-btn.down { left:56px; bottom:0; }
    .dpad-btn.left { left:0; top:56px; }
    .dpad-btn.right { right:0; top:56px; }
    .dpad-center { position:absolute; left:56px; top:56px; width:62px; height:62px; border-radius:7px; background:radial-gradient(circle,#252525 0 18%,#151515 20% 100%); box-shadow:inset 2px 2px 4px rgba(255,255,255,.06); }
    .dpad-btn.pressed { transform:scale(.88); filter:brightness(1.6); }
    .nes-center { align-self:end; padding-bottom:24px; display:flex; flex-direction:column; align-items:center; gap:18px; text-align:center; }
    .reset-btn { position:relative; width:112px; height:34px; overflow:hidden; border:2px solid #6a1721; border-radius:9px; color:#701824; background:#d7cdb4; box-shadow:0 4px 0 #9b8f78,inset 0 1px 0 rgba(255,255,255,.55); font-size:10px; letter-spacing:1px; transition:transform .1s,box-shadow .1s,color .15s; }
    .reset-btn::before { content:""; position:absolute; inset:0; background:linear-gradient(90deg,#b8323f,#781722); transform:scaleX(0); transform-origin:left; }
    .reset-btn.holding { transform:translateY(3px); box-shadow:0 1px 0 #9b8f78; color:#fff1dd; }
    .reset-btn.holding::before { transform:scaleX(1); transition:transform 1s linear; }
    .reset-btn.sent { color:#fff1dd; background:#781722; transform:scale(.94); }
    .reset-btn span { position:relative; z-index:1; }
    .system-buttons { display:flex; justify-content:center; gap:22px; padding:14px 18px; border-radius:28px; background:rgba(120,36,44,.22); box-shadow:inset 0 2px 6px rgba(79,25,31,.25); }
    .system-btn-wrap { display:flex; flex-direction:column; align-items:center; gap:7px; color:#6a1721; font-size:10px; font-weight:900; letter-spacing:1px; }
    .system-btn { width:57px; height:23px; border:3px solid #352b29; border-radius:999px; background:linear-gradient(#353535,#171717); box-shadow:0 4px 0 #160f0e,inset 0 1px 2px rgba(255,255,255,.18); transition:transform .08s,box-shadow .08s; }
    .system-btn.pressed { transform:translateY(4px) scale(.96); box-shadow:0 0 0 #160f0e; }
    .action-area { display:flex; justify-content:center; align-items:center; gap:26px; transform:rotate(-7deg); }
    .action-wrap { display:flex; flex-direction:column; align-items:center; gap:8px; color:#701824; font-size:17px; font-weight:900; }
    .action-well { padding:9px; border-radius:50%; background:#c7bca5; box-shadow:inset 0 3px 6px rgba(60,40,35,.25); }
    .action-btn { width:72px; height:72px; border:4px solid #4a1017; border-radius:50%; color:#f2d9ca; font-size:26px; background:radial-gradient(circle at 35% 28%,#bd4650,#741722 65%,#4e0c14); box-shadow:0 7px 0 #3b0b11,0 10px 12px rgba(0,0,0,.3),inset 2px 2px 4px rgba(255,255,255,.18); transition:transform .08s,box-shadow .08s; }
    .action-btn.pressed { transform:translateY(6px) scale(.94); box-shadow:0 1px 0 #3b0b11,0 3px 5px rgba(0,0,0,.3); }
    .nes-status { position:absolute; bottom:9px; left:50%; transform:translateX(-50%); color:rgba(255,239,211,.72); font-size:10px; font-weight:700; letter-spacing:.5px; pointer-events:none; }
    @media (max-height:430px) {
      .title { margin-top:4px; }
      #debug { top:25px; font-size:12px; }
      .gyro-panel { height:62px; margin-top:30px; padding:7px 12px; }
      #gyroToggle { height:40px; }
      .gyro-readout { margin-bottom:4px; }
      .container { height:calc(100vh - 104px); transform:translateY(4px); }
      .camera-panel { flex-basis:220px; width:min(220px,31vw); border-radius:14px; }
      .label { margin-top:4px; font-size:12px; }
      .nes-shell { height:calc(100vh - 45px); min-height:225px; padding:13px 20px; border-radius:32px; }
      .nes-face { padding:12px 18px; gap:12px; }
      .nes-brand { top:18px; }
      .dpad { transform:scale(.78); }
      .action-area { gap:12px; }
      .action-btn { width:58px; height:58px; }
      .system-buttons { gap:12px; padding:10px 12px; }
      .system-btn { width:46px; }
      .nes-center { padding-bottom:10px; gap:10px; }
      .reset-btn { width:96px; height:28px; }
      .nes-status { bottom:3px; }
      #robotPage.pc-gamepad .container { height:calc(100vh - 104px); grid-template-rows:minmax(130px,1fr) 88px; padding:2px 18px 5px; column-gap:52px; }
      #robotPage.pc-gamepad .camera-panel { width:min(300px,46vw); }
      #robotPage.pc-gamepad canvas { width:76px; height:76px; }
      #robotPage.pc-gamepad .label { margin-top:1px; font-size:9px; }
    }
    @media (max-width:560px) and (orientation:portrait) {
      .gyro-panel { margin-top:40px; }
      .container { box-sizing:border-box; height:calc(100vh - 132px); display:grid; grid-template-columns:1fr 1fr; grid-template-rows:minmax(125px,1fr) auto; gap:4px 8px; padding:2px 8px 8px; transform:none; }
      .camera-panel { grid-column:1 / 3; width:min(210px,62vw); align-self:center; justify-self:center; }
      .joy-wrap { align-self:end; }
      .nes-shell { width:98vw; height:72vh; min-height:390px; padding:14px; border-radius:30px; }
      .nes-face { grid-template-columns:1fr 1fr; grid-template-rows:1fr auto; padding:26px 12px 12px; }
      .nes-center { grid-column:1 / 3; grid-row:2; padding-bottom:0; gap:8px; }
      .dpad { transform:scale(.78); }
      .action-area { gap:10px; }
      .action-btn { width:58px; height:58px; }
      .system-buttons { padding:8px 15px; }
      #robotPage.pc-gamepad .container { width:100%; grid-template-rows:minmax(180px,1fr) 116px; padding:4px 12px 9px; column-gap:18px; }
      #robotPage.pc-gamepad .camera-panel { width:min(360px,88vw); }
      #robotPage.pc-gamepad canvas { width:98px; height:98px; }
    }
  </style>
</head>
<body>
  <button id="inputSwitch" class="input-switch" type="button" title="Switch between touch joysticks and a controller connected to this computer">INPUT: TOUCH</button>
  <button id="modeSwitch" class="mode-switch" type="button">OPEN NES</button>
  <main id="robotPage" class="mode-page">
    <div class="title">ESP32 Web Joystick Controller</div>
    <div id="debug">L: (0,0) | R: (0,0)</div>
    <div class="gyro-panel">
      <button id="gyroToggle" type="button" aria-pressed="false">GYRO OFF</button>
      <div class="gyro-slider-wrap">
        <div class="gyro-readout"><span>Spin control</span><span id="gyroValue">IDLE 0%</span></div>
        <input id="gyroSlider" type="range" min="-100" max="100" value="0" step="1" aria-label="Gyro direction and speed">
        <div class="gyro-scale"><span>CCW -100</span><span>0</span><span>CW +100</span></div>
      </div>
    </div>
    <div class="container">
      <div class="joy-wrap">
         <canvas id="joyL" width="160" height="160"></canvas>
         <div class="label">Left Stick</div>
      </div>
      <section class="camera-panel" aria-label="OV5640 live preview">
        <img id="cameraImage" alt="OV5640 live preview">
        <div id="cameraPlaceholder">CAMERA WAIT<br>Connecting video link...</div>
        <div class="camera-badge"><span id="cameraState">WAIT</span><span id="cameraStats">-- FPS</span></div>
      </section>
      <div class="joy-wrap">
         <canvas id="joyR" width="160" height="160"></canvas>
         <div class="label">Right Stick</div>
      </div>
    </div>
  </main>

  <main id="nesPage" class="mode-page nes-page" hidden>
    <section class="nes-shell" aria-label="NES virtual controller">
      <div class="nes-brand">ESP32 · NES CONTROLLER</div>
      <div class="nes-face">
        <div class="dpad" aria-label="Direction pad">
          <button class="nes-btn dpad-btn up" data-nes-bit="16" aria-label="Up">▲</button>
          <button class="nes-btn dpad-btn down" data-nes-bit="32" aria-label="Down">▼</button>
          <button class="nes-btn dpad-btn left" data-nes-bit="64" aria-label="Left">◀</button>
          <button class="nes-btn dpad-btn right" data-nes-bit="128" aria-label="Right">▶</button>
          <div class="dpad-center"></div>
        </div>
        <div class="nes-center">
          <button id="nesReset" class="nes-btn reset-btn" type="button" aria-label="Hold to reset game"><span>HOLD RESET</span></button>
          <div class="system-buttons">
            <label class="system-btn-wrap">SELECT<button class="nes-btn system-btn" data-nes-bit="4" aria-label="Select"></button></label>
            <label class="system-btn-wrap">START<button class="nes-btn system-btn" data-nes-bit="8" aria-label="Start"></button></label>
          </div>
        </div>
        <div class="action-area">
          <div class="action-wrap"><span>B</span><div class="action-well"><button class="nes-btn action-btn" data-nes-bit="2" aria-label="B">B</button></div></div>
          <div class="action-wrap"><span>A</span><div class="action-well"><button class="nes-btn action-btn" data-nes-bit="1" aria-label="A">A</button></div></div>
        </div>
      </div>
      <div id="nesStatus" class="nes-status">WAIT · BUTTONS 00</div>
    </section>
  </main>

  <script>
    let clx=0, cly=0, crx=0, cry=0;
    let gyroEnabled=false, gyroSignedSpeed=0, gyroDirty=true;
    let axesDirty=true, lastAxesSendMs=0, debugUpdatePending=false;
    let axesInFlight=false, axesSequence=0, axesSentAtMs=0;
    let axesTimeoutReported=false, axesSentWasZero=true;
    let lastRttMs=0, mergedAxesCount=0, ackTimeoutCount=0;
    let socketClaimed=false, superseded=false;
    let controlMode=0, nesButtons=0, nesSequence=0, lastNesSendMs=0;
    let inputMode=0, gamepadIndex=-1, gamepadState='TOUCH', gamepadName='';
    let inputHandoverPending=false;
    let leftJoy=null, rightJoy=null, gamepadLoopStarted=false;
    const nesPointers=new Map();
    const AXES_MIN_SEND_MS=20, AXES_KEEPALIVE_MS=100, AXES_ACK_TIMEOUT_MS=300;
    const WS_MAX_BUFFERED_BYTES=128;
    const NES_KEEPALIVE_MS=50, MODE_MECANUM=0, MODE_NES=1;
    const INPUT_TOUCH=0, INPUT_PC_GAMEPAD=1, GAMEPAD_DEADZONE=.12;
    const WS_MSG_ACK=0x41, WS_MSG_HELLO=0x48, WS_MSG_AXES=0x4A;
    const sessionId = (() => {
      if(window.crypto && window.crypto.getRandomValues) {
        const value = new Uint32Array(1);
        window.crypto.getRandomValues(value);
        return value[0] || 1;
      }
      return ((Date.now() ^ Math.floor(Math.random()*0xffffffff)) >>> 0) || 1;
    })();
    const debug = document.getElementById('debug');
    const robotPage = document.getElementById('robotPage');
    const nesPage = document.getElementById('nesPage');
    const modeSwitch = document.getElementById('modeSwitch');
    const inputSwitch = document.getElementById('inputSwitch');
    const nesStatus = document.getElementById('nesStatus');
    const nesReset = document.getElementById('nesReset');
    let nesResetTimer=null, nesResetPointer=null, nesResetSequence=0;
    const gyroToggle = document.getElementById('gyroToggle');
    const gyroSlider = document.getElementById('gyroSlider');
    const gyroValue = document.getElementById('gyroValue');
    const cameraImage = document.getElementById('cameraImage');
    const cameraPlaceholder = document.getElementById('cameraPlaceholder');
    const cameraState = document.getElementById('cameraState');
    const cameraStats = document.getElementById('cameraStats');
    let cameraFrameSequence=-1, cameraFrameBuffer=null, cameraFrameOffset=0;
    let cameraDisplayBusy=false, cameraPendingFrame=null, cameraCurrentUrl=null;
    let cameraLastFrameAt=0, cameraWaitStartedAt=performance.now();
    let cameraFps=0, cameraCompletedFrames=0, cameraWireSequence=0;

    function showCameraWait(title, detail) {
      if(cameraImage.classList.contains('ready')) return;
      cameraPlaceholder.style.display='flex';
      cameraPlaceholder.innerHTML=title+'<br>'+detail;
    }

    function resetCameraAssembly() {
      cameraFrameSequence=-1;
      cameraFrameBuffer=null;
      cameraFrameOffset=0;
    }

    function displayCameraFrame(frame, width, height, sequence) {
      const item={blob:new Blob([frame],{type:'image/jpeg'}),width,height,sequence};
      if(cameraDisplayBusy) {
        cameraPendingFrame=item;
        return;
      }

      cameraDisplayBusy=true;
      const url=URL.createObjectURL(item.blob);
      cameraImage.onload=() => {
        if(cameraCurrentUrl) URL.revokeObjectURL(cameraCurrentUrl);
        cameraCurrentUrl=url;
        cameraDisplayBusy=false;
        cameraImage.classList.add('ready');
        cameraPlaceholder.style.display='none';
        cameraState.textContent='LIVE';
        cameraState.classList.add('live');
        const now=performance.now();
        if(cameraLastFrameAt) {
          const instant=1000/Math.max(1,now-cameraLastFrameAt);
          cameraFps=cameraFps ? cameraFps*.75+instant*.25 : instant;
        }
        cameraLastFrameAt=now;
        cameraCompletedFrames++;
        cameraStats.textContent=`${cameraFps.toFixed(1)} FPS · #${item.sequence}`;
        sendCameraCredit();
        if(cameraPendingFrame) {
          const pending=cameraPendingFrame;
          cameraPendingFrame=null;
          displayCameraFrame(pending.blob, pending.width, pending.height, pending.sequence);
        }
      };
      cameraImage.onerror=() => {
        URL.revokeObjectURL(url);
        cameraDisplayBusy=false;
        cameraState.textContent='JPEG DROP';
        cameraState.classList.remove('live');
        sendCameraCredit();
        if(cameraPendingFrame) {
          const pending=cameraPendingFrame;
          cameraPendingFrame=null;
          displayCameraFrame(pending.blob, pending.width, pending.height, pending.sequence);
        }
      };
      cameraImage.src=url;
    }

    function sendCameraCredit() {
      if(cameraSocket && cameraSocket.readyState===WebSocket.OPEN &&
         cameraSocket.bufferedAmount===0) {
        cameraSocket.send(new Uint8Array([0x43]).buffer);
      }
    }

    function handleCameraPacket(message) {
      /* New firmware sends one complete JPEG per WebSocket message. Retain
       * the CAM1 chunk decoder below so a cached older ESP32 page remains
       * compatible during development. */
      if(message.length>=4 && message[0]===0xFF && message[1]===0xD8 &&
         message[message.length-2]===0xFF && message[message.length-1]===0xD9) {
        displayCameraFrame(message,320,240,++cameraWireSequence);
        return true;
      }
      if(message.length < 26 || message[0]!==0x43 || message[1]!==0x41 ||
         message[2]!==0x4D || message[3]!==0x31) return false;
      const view=new DataView(message.buffer,message.byteOffset,message.byteLength);
      const flags=message[5], sequence=view.getUint16(6,true);
      const offset=view.getUint32(8,true), total=view.getUint32(12,true);
      const width=view.getUint16(16,true), height=view.getUint16(18,true);
      const payloadLength=view.getUint16(20,true);
      if(message.length!==22+payloadLength+4 || !total || total>65536 ||
         !payloadLength || offset+payloadLength>total) {
        resetCameraAssembly();
        return true;
      }
      if(flags&1) {
        if(offset!==0) { resetCameraAssembly(); return true; }
        cameraFrameSequence=sequence;
        cameraFrameBuffer=new Uint8Array(total);
        cameraFrameOffset=0;
      }
      if(!cameraFrameBuffer || sequence!==cameraFrameSequence ||
         offset!==cameraFrameOffset || total!==cameraFrameBuffer.length) {
        resetCameraAssembly();
        cameraState.textContent='FRAME DROP';
        cameraState.classList.remove('live');
        return true;
      }
      cameraFrameBuffer.set(message.subarray(22,22+payloadLength),offset);
      cameraFrameOffset+=payloadLength;
      if(flags&2) {
        if(cameraFrameOffset===cameraFrameBuffer.length) {
          const completed=cameraFrameBuffer;
          resetCameraAssembly();
          displayCameraFrame(completed,width,height,sequence);
        } else {
          resetCameraAssembly();
        }
      }
      return true;
    }

    function updateGyroUI(animate=false) {
      const value = gyroSignedSpeed;
      const center = 50;
      const position = center + value * 0.5;
      let color = '#94a3b8';
      let glow = 'rgba(148,163,184,.32)';
      let background;

      if(value > 0) {
        color = '#38bdf8';
        glow = 'rgba(56,189,248,.48)';
        background = `linear-gradient(90deg,#2a303b 0%,#2a303b 50%,#38bdf8 50%,#2563eb ${position}%,#2a303b ${position}%,#2a303b 100%)`;
        gyroValue.textContent = `CW +${value}%`;
      } else if(value < 0) {
        color = '#f59e0b';
        glow = 'rgba(245,158,11,.48)';
        background = `linear-gradient(90deg,#2a303b 0%,#2a303b ${position}%,#f97316 ${position}%,#f59e0b 50%,#2a303b 50%,#2a303b 100%)`;
        gyroValue.textContent = `CCW ${value}%`;
      } else {
        background = 'linear-gradient(90deg,#2a303b 0%,#2a303b 48%,#94a3b8 48%,#94a3b8 52%,#2a303b 52%,#2a303b 100%)';
        gyroValue.textContent = 'IDLE 0%';
      }

      document.documentElement.style.setProperty('--gyro-color', color);
      document.documentElement.style.setProperty('--gyro-glow', glow);
      gyroSlider.style.background = background;
      gyroToggle.classList.toggle('on', gyroEnabled);
      gyroToggle.textContent = gyroEnabled ? 'GYRO ON' : 'GYRO OFF';
      gyroToggle.setAttribute('aria-pressed', gyroEnabled ? 'true' : 'false');
      if(animate) {
        gyroValue.classList.remove('pulse');
        void gyroValue.offsetWidth;
        gyroValue.classList.add('pulse');
        setTimeout(() => gyroValue.classList.remove('pulse'), 150);
      }
      updateDebug();
    }

    function queueGyroState() {
      gyroDirty = true;
      sendLatest();
      sendGyroState();
    }

    function updateNesStatus() {
      const state = socketReady() ? 'LINK' : 'WAIT';
      nesStatus.textContent = `${state} · BUTTONS ${nesButtons.toString(16).toUpperCase().padStart(2,'0')}`;
    }

    function sendMode() {
      if(!socketReady() || socket.bufferedAmount > WS_MAX_BUFFERED_BYTES) return;
      socket.send(new Uint8Array([0x4D, controlMode]).buffer);
    }

    function sendNesState(force=false) {
      if(controlMode !== MODE_NES || !socketReady()) return;
      const now = performance.now();
      if(!force && (now - lastNesSendMs < NES_KEEPALIVE_MS)) return;
      if(socket.bufferedAmount > WS_MAX_BUFFERED_BYTES) return;
      socket.send(new Uint8Array([0x4E, nesButtons, nesSequence++ & 0xFF]).buffer);
      lastNesSendMs = now;
      updateNesStatus();
    }

    function releaseAllNes(send=true) {
      cancelNesReset();
      nesPointers.clear();
      nesButtons = 0;
      document.querySelectorAll('[data-nes-bit]').forEach(btn => btn.classList.remove('pressed'));
      if(send) sendNesState(true);
      updateNesStatus();
    }

    function sendNesReset() {
      if(controlMode !== MODE_NES || !socketReady()) return false;
      if(socket.bufferedAmount > WS_MAX_BUFFERED_BYTES) return false;
      socket.send(new Uint8Array([0x52, nesResetSequence++ & 0xFF]).buffer);
      return true;
    }

    function cancelNesReset() {
      if(nesResetTimer !== null) {
        clearTimeout(nesResetTimer);
        nesResetTimer=null;
      }
      nesResetPointer=null;
      nesReset.classList.remove('holding','sent');
      nesReset.querySelector('span').textContent='HOLD RESET';
    }

    function setupNesReset() {
      nesReset.addEventListener('contextmenu', e => e.preventDefault());
      nesReset.addEventListener('pointerdown', e => {
        e.preventDefault();
        if(controlMode !== MODE_NES || nesResetPointer !== null) return;
        nesResetPointer=e.pointerId;
        nesReset.setPointerCapture(e.pointerId);
        nesReset.classList.remove('sent');
        // Restart the CSS fill animation even after consecutive resets.
        nesReset.classList.remove('holding');
        void nesReset.offsetWidth;
        nesReset.classList.add('holding');
        nesReset.querySelector('span').textContent='KEEP HOLDING';
        nesResetTimer=setTimeout(() => {
          nesResetTimer=null;
          nesReset.classList.remove('holding');
          nesReset.classList.add('sent');
          if(sendNesReset()) {
            nesReset.querySelector('span').textContent='RESET SENT';
            if(navigator.vibrate) navigator.vibrate([25,35,25]);
          } else {
            nesReset.querySelector('span').textContent='LINK LOST';
          }
        },1000);
      });
      const release=e => {
        if(nesResetPointer !== e.pointerId) return;
        e.preventDefault();
        cancelNesReset();
      };
      nesReset.addEventListener('pointerup',release);
      nesReset.addEventListener('pointercancel',release);
      nesReset.addEventListener('lostpointercapture',release);
    }

    function updateInputUI() {
      const pc=inputMode===INPUT_PC_GAMEPAD;
      robotPage.classList.toggle('pc-gamepad',pc);
      inputSwitch.classList.toggle('pc',pc);
      inputSwitch.classList.toggle('ready',pc && gamepadState==='READY');
      inputSwitch.classList.toggle('blocked',pc &&
        (gamepadState==='UNSUPPORTED' || gamepadState==='BLOCKED'));

      if(!pc) inputSwitch.textContent='INPUT: TOUCH';
      else if(gamepadState==='READY') inputSwitch.textContent='PC PAD: READY';
      else if(gamepadState==='BLOCKED') inputSwitch.textContent='PC PAD: BLOCKED';
      else if(gamepadState==='UNSUPPORTED') inputSwitch.textContent='PC PAD: UNSUPPORTED';
      else inputSwitch.textContent='PC PAD: PRESS BUTTON';

      inputSwitch.title=pc ?
        (gamepadName || 'Focus this page, then move a stick or press a controller button') :
        'Switch to a controller connected to this computer';
      if(leftJoy) leftJoy.sync();
      if(rightJoy) rightJoy.sync();
      updateDebug();
    }

    function setGamepadState(state,name='') {
      if(gamepadState===state && gamepadName===name) return;
      gamepadState=state;
      gamepadName=name;
      updateInputUI();
    }

    function releaseRobotAxes(send=true) {
      setAxesZero();
      if(leftJoy) leftJoy.sync();
      if(rightJoy) rightJoy.sync();
      if(send) sendLatest(true);
    }

    function setInputMode(nextMode) {
      if(nextMode!==INPUT_TOUCH && nextMode!==INPUT_PC_GAMEPAD) return;
      if(inputMode===nextMode) {
        updateInputUI();
        return;
      }

      /* Never carry a non-zero command across an input-source handover. */
      inputHandoverPending=true;
      releaseRobotAxes(true);
      if(leftJoy) leftJoy.cancel();
      if(rightJoy) rightJoy.cancel();
      inputMode=nextMode;
      gamepadIndex=-1;
      gamepadName='';
      gamepadState=nextMode===INPUT_PC_GAMEPAD ?
        (typeof navigator.getGamepads==='function' ? 'WAIT' : 'UNSUPPORTED') :
        'TOUCH';
      updateInputUI();
    }

    function readVisibleGamepads() {
      if(typeof navigator.getGamepads!=='function') {
        setGamepadState('UNSUPPORTED');
        return [];
      }
      try {
        return Array.from(navigator.getGamepads() || []);
      } catch(error) {
        setGamepadState('BLOCKED',error && error.name ? error.name : 'Gamepad API blocked');
        return [];
      }
    }

    function selectGamepad(pads) {
      if(gamepadIndex>=0) {
        const selected=pads[gamepadIndex];
        if(selected && selected.connected && selected.axes.length>=4) return selected;
      }
      const selected=pads.find(pad => pad && pad.connected &&
        pad.axes.length>=4 && pad.mapping==='standard') ||
        pads.find(pad => pad && pad.connected && pad.axes.length>=4);
      if(selected) gamepadIndex=selected.index;
      return selected || null;
    }

    function mapGamepadStick(rawX,rawY) {
      let x=Number.isFinite(rawX) ? Math.max(-1,Math.min(1,rawX)) : 0;
      let y=Number.isFinite(rawY) ? Math.max(-1,Math.min(1,rawY)) : 0;
      const magnitude=Math.min(1,Math.hypot(x,y));
      if(magnitude<=GAMEPAD_DEADZONE) return [0,0];
      const scaled=(magnitude-GAMEPAD_DEADZONE)/(1-GAMEPAD_DEADZONE);
      x=x/magnitude*scaled;
      y=y/magnitude*scaled;
      return [Math.round(x*100),Math.round(-y*100)];
    }

    function applyGamepadAxes(pad) {
      const left=mapGamepadStick(pad.axes[0],pad.axes[1]);
      const right=mapGamepadStick(pad.axes[2],pad.axes[3]);
      if(clx===left[0] && cly===left[1] && crx===right[0] && cry===right[1]) return;
      clx=left[0]; cly=left[1]; crx=right[0]; cry=right[1];
      if(axesInFlight) mergedAxesCount++;
      axesDirty=true;
      if(leftJoy) leftJoy.sync();
      if(rightJoy) rightJoy.sync();
      updateDebug();
      sendLatest();
    }

    function pollPcGamepad() {
      requestAnimationFrame(pollPcGamepad);
      if(inputMode!==INPUT_PC_GAMEPAD || controlMode!==MODE_MECANUM ||
         document.hidden || inputHandoverPending) return;

      const pad=selectGamepad(readVisibleGamepads());
      if(!pad) {
        const wasReady=gamepadState==='READY';
        gamepadIndex=-1;
        setGamepadState(gamepadState==='BLOCKED' || gamepadState==='UNSUPPORTED' ?
          gamepadState : 'WAIT');
        if(wasReady) releaseRobotAxes(true);
        return;
      }

      setGamepadState('READY',`${pad.id} | mapping:${pad.mapping || 'raw'} | axes:${pad.axes.length}`);
      applyGamepadAxes(pad);
    }

    function updateModeUI() {
      const nes = controlMode === MODE_NES;
      robotPage.hidden = nes;
      nesPage.hidden = !nes;
      inputSwitch.hidden = nes;
      modeSwitch.classList.toggle('nes', nes);
      modeSwitch.textContent = nes ? 'OPEN ROBOT' : 'OPEN NES';
      updateInputUI();
      updateDebug();
      updateNesStatus();
    }

    function switchMode() {
      if(controlMode === MODE_MECANUM) {
        // Stop every latched robot action before exposing the game controls.
        clx=0; cly=0; crx=0; cry=0;
        if(axesInFlight) mergedAxesCount++;
        axesDirty=true;
        gyroEnabled=false; gyroDirty=true;
        updateGyroUI(false);
        sendLatest(true);
        sendGyroState();
        controlMode=MODE_NES;
        releaseAllNes(false);
      } else {
        releaseAllNes(true);
        controlMode=MODE_MECANUM;
        axesDirty=true;
      }
      updateModeUI();
      sendMode();
      if(controlMode === MODE_NES) sendNesState(true);
      else sendLatest(true);
    }

    function setupNesButtons() {
      document.querySelectorAll('[data-nes-bit]').forEach(btn => {
        const bit = Number(btn.dataset.nesBit) & 0xFF;
        btn.addEventListener('contextmenu', e => e.preventDefault());
        btn.addEventListener('pointerdown', e => {
          e.preventDefault();
          if(controlMode !== MODE_NES) return;
          btn.setPointerCapture(e.pointerId);
          nesPointers.set(e.pointerId, {bit, btn});
          nesButtons |= bit;
          btn.classList.add('pressed');
          if(navigator.vibrate) navigator.vibrate(7);
          sendNesState(true);
        });
        const release = e => {
          const held = nesPointers.get(e.pointerId);
          if(!held) return;
          e.preventDefault();
          nesPointers.delete(e.pointerId);
          let rebuilt=0;
          nesPointers.forEach(item => rebuilt |= item.bit);
          nesButtons=rebuilt & 0xFF;
          if(!Array.from(nesPointers.values()).some(item => item.btn === held.btn)) {
            held.btn.classList.remove('pressed');
          }
          sendNesState(true);
        };
        btn.addEventListener('pointerup', release);
        btn.addEventListener('pointercancel', release);
        btn.addEventListener('lostpointercapture', release);
      });
    }

    modeSwitch.addEventListener('click', switchMode);
    inputSwitch.addEventListener('click', () => {
      setInputMode(inputMode===INPUT_TOUCH ? INPUT_PC_GAMEPAD : INPUT_TOUCH);
    });
    window.addEventListener('gamepadconnected', event => {
      if(inputMode!==INPUT_PC_GAMEPAD) return;
      gamepadIndex=event.gamepad.index;
      setGamepadState('READY',`${event.gamepad.id} | mapping:${event.gamepad.mapping || 'raw'} | axes:${event.gamepad.axes.length}`);
    });
    window.addEventListener('gamepaddisconnected', event => {
      if(inputMode!==INPUT_PC_GAMEPAD || event.gamepad.index!==gamepadIndex) return;
      gamepadIndex=-1;
      setGamepadState('WAIT');
      releaseRobotAxes(true);
    });
    window.addEventListener('blur', () => {
      if(controlMode === MODE_NES) releaseAllNes(true);
      else if(inputMode===INPUT_PC_GAMEPAD) releaseRobotAxes(true);
    });
    document.addEventListener('visibilitychange', () => {
      if(document.hidden && controlMode === MODE_NES) releaseAllNes(true);
      else if(document.hidden && inputMode===INPUT_PC_GAMEPAD) releaseRobotAxes(true);
    });

    gyroToggle.addEventListener('click', () => {
      gyroEnabled = !gyroEnabled;
      updateGyroUI(true);
      queueGyroState();
    });

    gyroSlider.addEventListener('input', () => {
      gyroSignedSpeed = Math.max(-100, Math.min(100, Number(gyroSlider.value) || 0));
      updateGyroUI(true);
      queueGyroState();
    });
    
    function initJoy(canvasId, isLeft) {
      const canvas = document.getElementById(canvasId);
      const ctx = canvas.getContext('2d');
      const radius = canvas.width / 2;
      const maxDist = radius - 30; // knob radius
      
      let cx = radius, cy = radius;
      let kx = cx, ky = cy;
      let active = false;
      let touchId = null;

      function sync() {
        const x=isLeft ? clx : crx;
        const y=isLeft ? cly : cry;
        kx=cx+(Math.max(-100,Math.min(100,x))/100)*maxDist;
        ky=cy-(Math.max(-100,Math.min(100,y))/100)*maxDist;
        draw();
      }

      function cancel() {
        active=false;
        touchId=null;
      }

      function draw() {
        ctx.clearRect(0, 0, canvas.width, canvas.height);
        
        // Draw center crosshair
        ctx.beginPath();
        ctx.moveTo(cx - 10, cy); ctx.lineTo(cx + 10, cy);
        ctx.moveTo(cx, cy - 10); ctx.lineTo(cx, cy + 10);
        ctx.strokeStyle = 'rgba(255,255,255,0.3)';
        ctx.lineWidth = 2;
        ctx.stroke();

        // Draw boundary
        ctx.beginPath();
        ctx.arc(cx, cy, maxDist, 0, Math.PI*2);
        ctx.strokeStyle = 'rgba(255,255,255,0.5)';
        ctx.lineWidth = 2;
        ctx.stroke();
        
        // Draw knob
        ctx.beginPath();
        ctx.arc(kx, ky, 30, 0, Math.PI*2);
        ctx.fillStyle = isLeft ? '#00a8ff' : '#e84118';
        ctx.fill();
        ctx.lineWidth = 3;
        ctx.strokeStyle = '#fff';
        ctx.stroke();
        ctx.closePath();
      }

      function updatePos(e) {
        if(inputMode!==INPUT_TOUCH || inputHandoverPending) return;
        let touch = null;
        if(e.changedTouches) {
          for(let i=0; i<e.changedTouches.length; i++){
            if(e.changedTouches[i].identifier === touchId){
              touch = e.changedTouches[i];
              break;
            }
          }
        }
        if(!touch && e.clientX === undefined) return;

        const rect = canvas.getBoundingClientRect();
        let tx = ((touch ? touch.clientX : e.clientX) - rect.left) *
                 canvas.width / Math.max(1,rect.width);
        let ty = ((touch ? touch.clientY : e.clientY) - rect.top) *
                 canvas.height / Math.max(1,rect.height);

        let dx = tx - cx;
        let dy = ty - cy;
        let dist = Math.sqrt(dx*dx + dy*dy);

        if (dist > maxDist) {
          kx = cx + (dx / dist) * maxDist;
          ky = cy + (dy / dist) * maxDist;
        } else {
          kx = tx;
          ky = ty;
        }

        let outX = Math.round(((kx - cx) / maxDist) * 100);
        let outY = Math.round(((ky - cy) / maxDist) * -100); // Inverse Y
        
        if(isLeft) { clx = outX; cly = outY; }
        else { crx = outX; cry = outY; }
        if(axesInFlight) mergedAxesCount++;
        axesDirty = true;
        draw();
        updateDebug();
        sendLatest();
      }

      canvas.addEventListener('touchstart', e => {
        if(inputMode!==INPUT_TOUCH || inputHandoverPending) return;
        e.preventDefault();
        active = true;
        touchId = e.changedTouches[0].identifier;
        updatePos(e);
      }, {passive:false});

      canvas.addEventListener('touchmove', e => {
        e.preventDefault();
        if(active) updatePos(e);
      }, {passive:false});

      const end = (e) => {
        if(!active || inputMode!==INPUT_TOUCH || inputHandoverPending) return;
        e.preventDefault();
        active = false;
        touchId = null;
        kx = cx; ky = cy;
        if(isLeft) { clx = 0; cly = 0; }
        else { crx = 0; cry = 0; }
        if(axesInFlight) mergedAxesCount++;
        axesDirty = true;
        draw();
        updateDebug();
        sendLatest(true);
      };

      canvas.addEventListener('touchend', end);
      canvas.addEventListener('touchcancel', end);
      
      canvas.addEventListener('mousedown', e => {
        if(inputMode!==INPUT_TOUCH || inputHandoverPending) return;
        active = true;
        updatePos(e);
      });
      canvas.addEventListener('mousemove', e => { if(active) updatePos(e); });
      canvas.addEventListener('mouseup', end);
      canvas.addEventListener('mouseleave', end);
      
      draw();
      return {sync,cancel};
    }

    // Force initialization after page fully paints
    setTimeout(() => {
        leftJoy=initJoy('joyL', true);
        rightJoy=initJoy('joyR', false);
        setupNesButtons();
        setupNesReset();
        updateModeUI();
        if(!gamepadLoopStarted) {
          gamepadLoopStarted=true;
          requestAnimationFrame(pollPcGamepad);
        }
    }, 100);

    let socket = null;
    let reconnectTimer = null;
    let cameraSocket = null;
    let cameraReconnectTimer = null;

    function socketReady() {
      return socket && socket.readyState === WebSocket.OPEN &&
             socketClaimed && !superseded;
    }

    function updateDebug() {
      if(debugUpdatePending) return;
      debugUpdatePending = true;
      requestAnimationFrame(() => {
        debugUpdatePending = false;
        const state = superseded ? 'TAKEN' : (socketReady() ? 'LINK' : 'WAIT');
        const gyroState = gyroEnabled ? `G:${gyroSignedSpeed}` : 'G:OFF';
        const inputState = inputHandoverPending ? 'INPUT:ARM' :
          (inputMode===INPUT_PC_GAMEPAD ? `PAD:${gamepadState}` : 'TOUCH');
        const link = socketReady() ?
          `RTT:${Math.round(lastRttMs)}ms M:${mergedAxesCount} T:${ackTimeoutCount}` :
          'RTT:--';
        debug.innerText = `${state} ${link} | ${inputState} | ${gyroState} | L: (${clx}, ${cly}) | R: (${crx}, ${cry})`;
      });
    }

    function sendLatest(force=false) {
      if(controlMode !== MODE_MECANUM || !socketReady()) return;

      const now = performance.now();
      if(axesInFlight) return;
      if(!force && (now - lastAxesSendMs < AXES_MIN_SEND_MS)) return;
      if(!axesDirty && (now - lastAxesSendMs < AXES_KEEPALIVE_MS)) return;

      /* Keep exactly one axes frame in flight. New touch positions overwrite
       * the local pending state instead of forming a stale TCP queue. */
      if(socket.bufferedAmount !== 0) return;
      axesSequence = (axesSequence + 1) & 0xFFFF;
      const axes = new Int8Array([
        WS_MSG_AXES, axesSequence & 0xFF, (axesSequence >> 8) & 0xFF,
        clx, cly, crx, cry
      ]);
      socket.send(axes.buffer);
      axesInFlight = true;
      axesSentWasZero = clx===0 && cly===0 && crx===0 && cry===0;
      axesTimeoutReported = false;
      axesSentAtMs = now;
      axesDirty = false;
      lastAxesSendMs = now;
    }

    function sendGyroState() {
      if(controlMode !== MODE_MECANUM || !gyroDirty || !socketReady()) return;
      if(axesInFlight || socket.bufferedAmount !== 0) return;

      // This low-rate latched command is queued behind the joystick frame.
      const gyro = new Int8Array([0x47, gyroEnabled ? 1 : 0, gyroSignedSpeed]);
      socket.send(gyro.buffer);
      gyroDirty = false;
    }

    function sendHello() {
      if(!socket || socket.readyState !== WebSocket.OPEN) return;
      const hello = new Uint8Array([
        WS_MSG_HELLO,
        sessionId & 0xFF, (sessionId >>> 8) & 0xFF,
        (sessionId >>> 16) & 0xFF, (sessionId >>> 24) & 0xFF
      ]);
      socket.send(hello.buffer);
    }

    function setAxesZero() {
      clx=0; cly=0; crx=0; cry=0;
      if(axesInFlight) mergedAxesCount++;
      axesDirty=true;
      if(leftJoy) leftJoy.sync();
      if(rightJoy) rightJoy.sync();
      updateDebug();
    }

    function handleSocketMessage(event) {
      if(typeof event.data === 'string') {
        if(event.data === 'ACTIVE') {
          socketClaimed=true;
          axesInFlight=false;
          axesDirty=true;
          gyroDirty=true;
          sendMode();
          if(controlMode === MODE_NES) sendNesState(true);
          else {
            sendLatest(true);
            sendGyroState();
          }
        } else if(event.data === 'TAKEN_OVER') {
          superseded=true;
          socketClaimed=false;
          axesInFlight=false;
          if(controlMode === MODE_NES) releaseAllNes(false);
          else setAxesZero();
          if(socket) socket.close();
          if(cameraSocket) cameraSocket.close();
        }
        updateDebug();
        updateNesStatus();
        return;
      }

      const message = new Uint8Array(event.data);
      if(handleCameraPacket(message)) return;
      if(message.length === 3 && message[0] === WS_MSG_ACK) {
        const ackSequence = message[1] | (message[2] << 8);
        if(axesInFlight && ackSequence === axesSequence) {
          const acknowledgedZero=axesSentWasZero;
          lastRttMs = performance.now() - axesSentAtMs;
          axesInFlight=false;
          axesTimeoutReported=false;
          if(inputHandoverPending && acknowledgedZero) inputHandoverPending=false;
          updateDebug();
          if(inputHandoverPending) {
            /* Complete the mandatory zero-frame handover before allowing a
             * low-rate latched command to occupy the WebSocket. */
            if(axesDirty) sendLatest(true);
          } else {
            /* Do not starve a low-rate gyro change while the stick is moving. */
            sendGyroState();
            if(axesDirty) sendLatest(true);
          }
        }
      }
    }

    function scheduleReconnect() {
      if(reconnectTimer !== null || superseded || document.hidden) return;
      reconnectTimer = setTimeout(() => {
        reconnectTimer = null;
        connectSocket();
      }, 500);
    }

    function scheduleCameraReconnect() {
      if(cameraReconnectTimer!==null || superseded || document.hidden) return;
      cameraReconnectTimer=setTimeout(() => {
        cameraReconnectTimer=null;
        connectCameraSocket();
      },700);
    }

    function connectCameraSocket() {
      if(superseded || document.hidden || (cameraSocket &&
         (cameraSocket.readyState===WebSocket.OPEN ||
          cameraSocket.readyState===WebSocket.CONNECTING))) return;
      try {
        cameraSocket=new WebSocket('ws://'+window.location.hostname+':82/');
        cameraSocket.binaryType='arraybuffer';
        cameraSocket.onopen=() => {
          cameraWaitStartedAt=performance.now();
          cameraState.textContent='LINK';
          cameraState.classList.remove('live');
          showCameraWait('VIDEO LINK READY','Waiting for STM32 stream');
          sendCameraCredit();
        };
        cameraSocket.onmessage=event => {
          if(typeof event.data!=='string') {
            handleCameraPacket(new Uint8Array(event.data));
          }
        };
        cameraSocket.onclose=() => {
          cameraSocket=null;
          resetCameraAssembly();
          cameraState.textContent='WAIT';
          cameraState.classList.remove('live');
          showCameraWait('CAMERA WAIT','Reconnecting video link...');
          if(!superseded && !document.hidden) scheduleCameraReconnect();
        };
        cameraSocket.onerror=() => {
          if(cameraSocket) cameraSocket.close();
        };
      } catch(e) {
        cameraSocket=null;
        if(!superseded && !document.hidden) scheduleCameraReconnect();
      }
    }

    function connectSocket() {
      if(superseded || document.hidden) return;
      if(socket && (socket.readyState === WebSocket.OPEN ||
                    socket.readyState === WebSocket.CONNECTING)) return;

      try {
        /* Use the address that served this page. This works with the router
         * address, mDNS hostname and the 192.168.4.1 fallback AP. */
        socket = new WebSocket('ws://' + window.location.hostname + ':81/');
        socket.binaryType = 'arraybuffer';
        socket.onopen = () => {
          socketClaimed=false;
          axesInFlight=false;
          updateDebug();
          sendHello();
        };
        socket.onmessage = handleSocketMessage;
        socket.onclose = () => {
          socket = null;
          socketClaimed=false;
          axesInFlight=false;
          updateDebug();
          updateNesStatus();
          if(!superseded && !document.hidden) scheduleReconnect();
        };
        socket.onerror = () => {
          if(socket) socket.close();
        };
      } catch(e) {
        socket = null;
        if(!superseded && !document.hidden) scheduleReconnect();
      }
      updateDebug();
    }

    // Dirty axes are sent at up to 50 Hz; unchanged state is kept alive at
    // 10 Hz. Touch handlers also request an immediate latest-value send.
    setInterval(() => {
      if(axesInFlight && !axesTimeoutReported &&
         (performance.now() - axesSentAtMs >= AXES_ACK_TIMEOUT_MS)) {
        /* WebSocket runs over TCP, so replacing this sequence cannot recover
         * a lost packet: TCP is already retransmitting it.  More importantly,
         * an old ACK would then never match the newly in-flight sequence and
         * could create a permanent timeout/retry loop.  Report this sequence
         * once and keep waiting for its matching cumulative transport order;
         * the latest touch position remains merged in axesDirty. */
        ackTimeoutCount++;
        axesTimeoutReported=true;
        axesDirty=true;
        updateDebug();
        return;
      }
      if(controlMode === MODE_NES) sendNesState();
      else {
        sendLatest();
        sendGyroState();
      }
    }, 20);

    setInterval(() => {
      const now=performance.now();
      const videoLinked=(cameraSocket && cameraSocket.readyState===WebSocket.OPEN) ||
                        socketReady();
      if(controlMode===MODE_MECANUM && videoLinked &&
         ((cameraLastFrameAt && now-cameraLastFrameAt>2000) ||
          (!cameraLastFrameAt && now-cameraWaitStartedAt>2000))) {
        cameraState.textContent='WAIT';
        cameraState.classList.remove('live');
        cameraStats.textContent='-- FPS';
        showCameraWait('STM32 STREAM WAIT','Check LCD Command Control / UART5 TX');
      }
    },500);

    document.addEventListener('visibilitychange', () => {
      if(document.hidden) {
        if(controlMode === MODE_NES) releaseAllNes(false);
        else setAxesZero();
        if(socket) socket.close();
        if(cameraSocket) cameraSocket.close();
      } else if(!superseded) {
        connectSocket();
        connectCameraSocket();
      }
    });
    window.addEventListener('pagehide', () => {
      if(controlMode === MODE_NES) releaseAllNes(false);
      else setAxesZero();
      if(socket) socket.close();
      if(cameraSocket) cameraSocket.close();
    });
    connectSocket();
    connectCameraSocket();
    updateGyroUI(false);
    updateDebug();
  </script>
</body>
</html>
)rawliteral";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Expires", "0");
  server.sendHeader("Permissions-Policy", "gamepad=(self)");
  /* index_html lives in Flash. server.send() first materializes a large
   * String in the heap; after adding the 64 KiB JPEG assembler that temporary
   * allocation can fail and the browser receives an empty white page.
   * send_P() streams the PROGMEM payload without making that full RAM copy. */
  server.send_P(200, "text/html", index_html, sizeof(index_html) - 1U);
}

uint16_t cameraReadU16(const uint8_t* data) {
  return (uint16_t)data[0] | ((uint16_t)data[1] << 8U);
}

uint32_t cameraReadU32(const uint8_t* data) {
  return (uint32_t)data[0] |
         ((uint32_t)data[1] << 8U) |
         ((uint32_t)data[2] << 16U) |
         ((uint32_t)data[3] << 24U);
}

uint32_t cameraCrc32Software(const uint8_t* data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for(size_t index = 0; index < length; ++index) {
    crc ^= data[index];
    for(uint8_t bit = 0; bit < 8U; ++bit) {
      crc = (crc & 1U) ? ((crc >> 1U) ^ 0xEDB88320UL) : (crc >> 1U);
    }
  }
  return crc ^ 0xFFFFFFFFUL;
}

uint32_t cameraCrc32(const uint8_t* data, size_t length) {
  /* esp_rom_crc32_le(0, ...) implements the same reflected IEEE CRC-32 used
   * by STM32 (poly 0xEDB88320, init/xor-out 0xFFFFFFFF), but uses the ROM
   * lookup table instead of eight C iterations for every incoming byte. */
  if(cameraRomCrcValid) {
    return esp_rom_crc32_le(0U, data, (uint32_t)length);
  }
  return cameraCrc32Software(data, length);
}

void cameraUartReceiveError(hardwareSerial_error_t error) {
  cameraUartLastDriverError = (uint8_t)error;
  switch(error) {
    case UART_FIFO_OVF_ERROR:
      cameraUartFifoOverflowCount++;
      break;
    case UART_BUFFER_FULL_ERROR:
      cameraUartBufferFullCount++;
      break;
    case UART_FRAME_ERROR:
      cameraUartFrameErrorCount++;
      break;
    case UART_PARITY_ERROR:
      cameraUartParityErrorCount++;
      break;
    case UART_BREAK_ERROR:
      cameraUartBreakErrorCount++;
      break;
    default:
      break;
  }
}

void resetCameraUartParser() {
  cameraUartRxState = CAMERA_UART_FIND_MAGIC;
  cameraUartPacketIndex = 0U;
  cameraUartExpectedLength = 0U;
  cameraUartMagicMatched = 0U;
}

void sendCameraFrameAck(uint16_t sequence) {
  const uint8_t txBuf[] = {
    HEADER_1, HEADER_2, 0x08, CAMERA_UART_DEVICE_ID,
    CAMERA_UART_CMD_FRAME_ACK,
    (uint8_t)(sequence & 0xFFU), (uint8_t)((sequence >> 8U) & 0xFFU),
    FRAME_END
  };
  Serial1.write(txBuf, sizeof(txBuf));
}

void deferCameraFrameAck(uint16_t sequence, uint32_t now) {
  cameraFrameAckPending = true;
  cameraFrameAckSequence = sequence;
  cameraFrameAckStartedMs = now;
  cameraFrameAckDeferredCount++;
}

void releaseCameraFrameAck() {
  if(!cameraFrameAckPending) return;
  sendCameraFrameAck(cameraFrameAckSequence);
  cameraFrameAckPending = false;
  cameraFrameAckReleasedCount++;
}

void sendCameraStreamCommand(bool enabled) {
  const uint8_t txBuf[] = {
    HEADER_1, HEADER_2, 0x07, CAMERA_UART_DEVICE_ID,
    CAMERA_UART_CMD_ENABLE, enabled ? (uint8_t)1U : (uint8_t)0U,
    FRAME_END
  };
  Serial1.write(txBuf, sizeof(txBuf));
  cameraStreamRequested = enabled;
  lastCameraRequestMs = millis();
}

void serviceCameraStreamRequest(uint32_t now) {
  /* The LCD Command Control page remains the real safety gate on STM32.
   * Do not make camera startup depend on both browser sockets being claimed:
   * some mobile browsers or routers temporarily block the secondary port. */
  const bool wanted = (activeCameraWebSocketClient != 0xFFU) &&
                      (controlMode == CONTROL_MODE_MECANUM);
  if(wanted) {
    cameraTurboLastWantedMs = now;
  }

  /* Keep the camera radio session alive across a brief port-82 reconnect.
   * Otherwise every video socket retry would tear down and recreate NimBLE. */
  const bool turboWanted = wanted ||
      (wifiCameraTurbo && (now - cameraTurboLastWantedMs < 2000U));
  if(turboWanted && !wifiCameraTurbo &&
     (now - cameraTurboLastAttemptMs >= 2000U)) {
    cameraTurboLastAttemptMs = now;
    bool radioReady = !gamepadBleEnabled;
    if(gamepadBleEnabled && !flydigi_gamepad::isConnected()) {
      radioReady = flydigi_gamepad::setRadioSuspended(true);
    }

    if(radioReady) {
      wifiCameraTurbo = setWifiHighPerformanceMode(true);
      if(!wifiCameraTurbo && gamepadBleEnabled &&
         flydigi_gamepad::isRadioSuspended()) {
        (void)setWifiHighPerformanceMode(false);
        (void)flydigi_gamepad::setRadioSuspended(false);
      }
    }
  } else if(!turboWanted && wifiCameraTurbo) {
    /* Ordering is safety-critical on this old IDF: MIN_MODEM first, then BLE.
     * Reversing it reproduces the coexistence abort seen at boot. */
    if(setWifiHighPerformanceMode(false)) {
      if(gamepadBleEnabled && flydigi_gamepad::isRadioSuspended()) {
        (void)flydigi_gamepad::setRadioSuspended(false);
      }
      wifiCameraTurbo = false;
    }
  }
  if(wanted) {
    if(!cameraStreamRequested ||
       (now - lastCameraRequestMs >= CAMERA_REQUEST_INTERVAL_MS)) {
      sendCameraStreamCommand(true);
    }
  } else if(cameraStreamRequested) {
    sendCameraStreamCommand(false);
  }
}

void resetCameraJpegAssembly(bool countDrop) {
  if(countDrop && cameraJpegAssembling) cameraJpegAssemblyDropCount++;
  cameraJpegExpectedSize = 0U;
  cameraJpegOffset = 0U;
  cameraJpegSequence = 0U;
  cameraJpegAssembling = false;
}

void forwardCompleteCameraJpeg(uint32_t now, uint16_t sequence) {
  const uint32_t size = cameraJpegExpectedSize;

  if((activeCameraWebSocketClient == 0xFFU) || cameraWebTxActive ||
     !cameraBrowserCredit ||
     (now - cameraLastWebFrameMs < CAMERA_WEB_FRAME_INTERVAL_MS)) {
    cameraJpegThrottleDropCount++;
    return;
  }

  cameraWebTxHeader[0] = 0x82U; /* FIN + binary frame. */
  if(size < 126U) {
    cameraWebTxHeader[1] = (uint8_t)size;
    cameraWebTxHeaderSize = 2U;
  } else if(size <= 0xFFFFU) {
    cameraWebTxHeader[1] = 126U;
    cameraWebTxHeader[2] = (uint8_t)(size >> 8U);
    cameraWebTxHeader[3] = (uint8_t)size;
    cameraWebTxHeaderSize = 4U;
  } else {
    cameraWebTxHeader[1] = 127U;
    cameraWebTxHeader[2] = 0U;
    cameraWebTxHeader[3] = 0U;
    cameraWebTxHeader[4] = 0U;
    cameraWebTxHeader[5] = 0U;
    cameraWebTxHeader[6] = (uint8_t)(size >> 24U);
    cameraWebTxHeader[7] = (uint8_t)(size >> 16U);
    cameraWebTxHeader[8] = (uint8_t)(size >> 8U);
    cameraWebTxHeader[9] = (uint8_t)size;
    cameraWebTxHeaderSize = 10U;
  }

  cameraWebTxClient = activeCameraWebSocketClient;
  cameraWebTxHeaderOffset = 0U;
  cameraWebTxSize = size;
  cameraWebTxOffset = 0U;
  cameraWebTxStartedMs = now;
  cameraWebTxLastProgressMs = now;
  cameraWebTxLastChunkMs = 0U;
  cameraBrowserCredit = false;
  cameraWebTxActive = true;
  /* The STM32 keeps its next captured frame ready until this ACK. Releasing
   * it only after TCP has copied the current JPEG prevents the single ESP32
   * frame buffer from forcing an alternating send/drop pattern. */
  deferCameraFrameAck(sequence, now);
}

void serviceCameraWebTx() {
  const uint8_t* data;
  uint32_t remaining;
  int sent;
  const uint32_t now = millis();

  if(!cameraWebTxActive) return;
  if((activeCameraWebSocketClient != cameraWebTxClient) ||
     !cameraWebSocket.clientIsConnected(cameraWebTxClient)) {
    cameraWebTxActive = false;
    cameraWebSocketDropCount++;
    releaseCameraFrameAck();
    return;
  }

  /* A camera frame may safely remain partially transmitted because no other
   * data is put on port 82 until it completes.  Yield this shared radio just
   * after a control frame arrives, allowing the tiny ACK on port 81 to leave
   * before another JPEG TCP segment is queued. */
  if((int32_t)(cameraControlGuardUntilMs - now) > 0) return;

  if(cameraWebTxHeaderOffset < cameraWebTxHeaderSize) {
    data = &cameraWebTxHeader[cameraWebTxHeaderOffset];
    remaining = cameraWebTxHeaderSize - cameraWebTxHeaderOffset;
  } else {
    if(now - cameraWebTxLastChunkMs < CAMERA_WEB_CHUNK_INTERVAL_MS) return;
    data = &cameraJpegFrame[cameraWebTxOffset];
    remaining = cameraWebTxSize - cameraWebTxOffset;
    if(remaining > CAMERA_WEB_TX_CHUNK_BYTES) {
      remaining = CAMERA_WEB_TX_CHUNK_BYTES;
    }
  }

  sent = cameraWebSocket.sendRawNonBlocking(cameraWebTxClient,
                                             data, remaining);
  if(sent > 0) {
    if((uint32_t)sent < remaining) cameraWebTxPartialWriteCount++;
    cameraWebTxLastProgressMs = now;
    if(cameraWebTxHeaderOffset < cameraWebTxHeaderSize) {
      cameraWebTxHeaderOffset += (uint8_t)sent;
    } else {
      cameraWebTxOffset += (uint32_t)sent;
      cameraWebTxLastChunkMs = now;
      if(cameraWebTxOffset >= cameraWebTxSize) {
        cameraWebTxActive = false;
        cameraLastWebFrameMs = now;
        cameraWebTxLastDurationMs = now - cameraWebTxStartedMs;
        if(cameraWebTxLastDurationMs > cameraWebTxMaxDurationMs) {
          cameraWebTxMaxDurationMs = cameraWebTxLastDurationMs;
        }
        cameraJpegWebSentCount++;
        releaseCameraFrameAck();
      }
    }
  } else {
    if(sent == 0) cameraWebTxWouldBlockCount++;
    if((sent >= 0) &&
       (now - cameraWebTxLastProgressMs < 1000U)) return;
    /* A partial WebSocket frame cannot be abandoned on a live TCP stream.
     * Close only the video socket; the independent control socket remains
     * responsive and the browser will reconnect port 82. */
    const uint8_t failedClient = cameraWebTxClient;
    cameraWebTxActive = false;
    cameraWebSocketDropCount++;
    cameraWebSocket.dropClientNow(failedClient);
    releaseCameraFrameAck();
  }
}

void handleCameraUartPacket() {
  const uint8_t flags = cameraUartPacket[5];
  const uint16_t sequence = cameraReadU16(&cameraUartPacket[6]);
  const uint32_t offset = cameraReadU32(&cameraUartPacket[8]);
  const uint32_t total = cameraReadU32(&cameraUartPacket[12]);
  const uint16_t payloadLength = cameraReadU16(&cameraUartPacket[20]);
  const uint32_t receivedCrc = cameraReadU32(
      &cameraUartPacket[CAMERA_HEADER_SIZE + payloadLength]);
  const uint32_t calculatedCrc = cameraCrc32(
      &cameraUartPacket[4], CAMERA_HEADER_SIZE - 4U + payloadLength);

  if(receivedCrc != calculatedCrc) {
    cameraUartCrcErrorCount++;
    return;
  }

  cameraUartPacketCount++;
  cameraLastPacketMs = millis();

  if((flags & 0x01U) != 0U) {
    const bool wasAssembling = cameraJpegAssembling;
    cameraJpegStartCount++;
    if(wasAssembling) cameraJpegStartOverlapCount++;
    resetCameraJpegAssembly(wasAssembling);
    if(cameraWebTxActive) {
      cameraJpegStartTxSuppressedCount++;
    } else if(millis() - cameraLastWebFrameMs <
              CAMERA_WEB_FRAME_INTERVAL_MS) {
      cameraJpegStartRateSuppressedCount++;
    } else if((offset == 0U) && (total >= 4U) &&
              (total <= CAMERA_MAX_FRAME_SIZE)) {
      cameraJpegExpectedSize = total;
      cameraJpegSequence = sequence;
      cameraJpegAssembling = true;
    }
  }

  if(cameraJpegAssembling &&
     (sequence == cameraJpegSequence) &&
     (total == cameraJpegExpectedSize) &&
     (offset == cameraJpegOffset) &&
     (offset + payloadLength <= cameraJpegExpectedSize)) {
    memcpy(&cameraJpegFrame[offset],
           &cameraUartPacket[CAMERA_HEADER_SIZE], payloadLength);
    cameraJpegOffset += payloadLength;

    if((flags & 0x02U) != 0U) {
      uint32_t jpegSize = cameraJpegExpectedSize;
      uint8_t trailingPadding = 0U;
      /* Older STM32 firmware transported the 0..31 zero bytes added solely
       * for its hardware JPEG DMA alignment. Accept that stream as well, but
       * never forward the padding after the JPEG FFD9 marker to a browser. */
      while((jpegSize >= 2U) && (trailingPadding <= 31U) &&
            !((cameraJpegFrame[jpegSize - 2U] == 0xFFU) &&
              (cameraJpegFrame[jpegSize - 1U] == 0xD9U))) {
        jpegSize--;
        trailingPadding++;
      }
      const bool complete = (cameraJpegOffset == cameraJpegExpectedSize) &&
                            (cameraJpegFrame[0] == 0xFFU) &&
                            (cameraJpegFrame[1] == 0xD8U) &&
                            (jpegSize >= 4U) &&
                            (cameraJpegFrame[jpegSize - 2U] == 0xFFU) &&
                            (cameraJpegFrame[jpegSize - 1U] == 0xD9U);
      if(complete) {
        cameraJpegExpectedSize = jpegSize;
        cameraJpegCompletedCount++;
        if(controlMode == CONTROL_MODE_MECANUM) {
          forwardCompleteCameraJpeg(millis(), sequence);
        }
        resetCameraJpegAssembly(false);
      } else {
        cameraJpegMarkerDropCount++;
        cameraJpegLastDropSequence = sequence;
        cameraJpegLastExpectedOffset = cameraJpegExpectedSize;
        cameraJpegLastReceivedOffset = cameraJpegOffset;
        resetCameraJpegAssembly(true);
      }
    }
  } else if(cameraJpegAssembling) {
    /* Once one UART chunk is absent, discard the rest of that JPEG. Sending
     * later chunks would only create a browser-side FRAME DROP storm. */
    cameraJpegOrderDropCount++;
    cameraJpegLastDropSequence = sequence;
    cameraJpegLastExpectedOffset = cameraJpegOffset;
    cameraJpegLastReceivedOffset = offset;
    resetCameraJpegAssembly(true);
  }

  /* A frame accepted for WebSocket transmission is acknowledged only after
   * TCP has copied it. Rejected/invalid frames are acknowledged immediately
   * so the STM32 producer can recover without waiting for its timeout. */
  if((flags & 0x02U) != 0U) {
    if(!cameraFrameAckPending ||
       (cameraFrameAckSequence != sequence)) {
      sendCameraFrameAck(sequence);
    }
  }
}

void consumeCameraUartByte(uint8_t value) {
  if(cameraUartRxState == CAMERA_UART_FIND_MAGIC) {
    if(value == CAMERA_MAGIC[cameraUartMagicMatched]) {
      cameraUartPacket[cameraUartMagicMatched++] = value;
      if(cameraUartMagicMatched == sizeof(CAMERA_MAGIC)) {
        cameraUartPacketIndex = sizeof(CAMERA_MAGIC);
        cameraUartRxState = CAMERA_UART_READ_HEADER;
      }
    } else {
      cameraUartMagicMatched = (value == CAMERA_MAGIC[0]) ? 1U : 0U;
      if(cameraUartMagicMatched != 0U) cameraUartPacket[0] = value;
    }
    return;
  }

  cameraUartPacket[cameraUartPacketIndex++] = value;
  if(cameraUartRxState == CAMERA_UART_READ_HEADER &&
     cameraUartPacketIndex == CAMERA_HEADER_SIZE) {
    const uint8_t flags = cameraUartPacket[5];
    const uint32_t offset = cameraReadU32(&cameraUartPacket[8]);
    const uint32_t total = cameraReadU32(&cameraUartPacket[12]);
    const uint16_t width = cameraReadU16(&cameraUartPacket[16]);
    const uint16_t height = cameraReadU16(&cameraUartPacket[18]);
    const uint16_t payloadLength = cameraReadU16(&cameraUartPacket[20]);
    const bool valid = cameraUartPacket[4] == 1U &&
                       payloadLength > 0U &&
                       payloadLength <= CAMERA_MAX_PAYLOAD &&
                       total >= 4U && total <= CAMERA_MAX_FRAME_SIZE &&
                       offset < total && offset + payloadLength <= total &&
                       width > 0U && height > 0U &&
                       (((flags & 0x01U) == 0U) || offset == 0U) &&
                       (((flags & 0x02U) == 0U) ||
                        (offset + payloadLength == total));
    if(!valid) {
      cameraUartFormatErrorCount++;
      resetCameraUartParser();
      return;
    }
    cameraUartExpectedLength = CAMERA_HEADER_SIZE + payloadLength +
                               CAMERA_CRC_SIZE;
    cameraUartRxState = CAMERA_UART_READ_PACKET;
  }

  if(cameraUartRxState == CAMERA_UART_READ_PACKET &&
     cameraUartPacketIndex == cameraUartExpectedLength) {
    handleCameraUartPacket();
    resetCameraUartParser();
  }
}

void processCameraUart() {
  /* HardwareSerial::available() and read() both enter the IDF UART mutex.
   * Calling both for every byte at 2.5 Mb/s caused roughly 500k driver calls
   * per second, starving WebSocket ACK handling and eventually overflowing
   * UART RX. Read a large block under one driver lock, then parse from RAM. */
  uint16_t budget = CAMERA_UART_READ_BUDGET;

  while(budget != 0U) {
    const int available = Serial1.available();
    if(available <= 0) break;

    if((uint32_t)available > cameraUartMaxBacklog) {
      cameraUartMaxBacklog = (uint32_t)available;
    }

    size_t requested = (size_t)available;
    if(requested > sizeof(cameraUartReadBuffer)) {
      requested = sizeof(cameraUartReadBuffer);
    }
    if(requested > budget) requested = budget;

    const size_t received = Serial1.read(cameraUartReadBuffer, requested);
    if(received == 0U) break;

    cameraUartBytesRead += (uint32_t)received;
    cameraUartBulkReadCount++;
    budget -= (uint16_t)received;
    for(size_t index = 0U; index < received; ++index) {
      consumeCameraUartByte(cameraUartReadBuffer[index]);
    }
  }
}

void setJoystickValues(int newLx, int newLy, int newRx, int newRy) {
  lx = constrain(newLx, -100, 100);
  ly = constrain(newLy, -100, 100);
  rx = constrain(newRx, -100, 100);
  ry = constrain(newRy, -100, 100);
}

void setGyroControl(bool enabled, int signedSpeed) {
  const int8_t clampedSpeed = (int8_t)constrain(signedSpeed, -100, 100);

  if((gyroEnabled != enabled) || (gyroSignedSpeed != clampedSpeed)) {
    gyroEnabled = enabled;
    gyroSignedSpeed = clampedSpeed;
    gyroCommandDirty = true;
    Serial.printf("[Gyro] %s value=%d\n", gyroEnabled ? "ON" : "OFF", gyroSignedSpeed);
  }
}

void setNesButtons(uint8_t buttons, uint8_t sequence) {
  nesButtons = buttons;
  nesSequence = sequence;
  lastNesUpdateMs = millis();
  nesCommandDirty = true;
}

void clearNesButtons(const char* reason) {
  if(nesButtons != 0U) {
    Serial.printf("[Safety] NES buttons released: %s\n", reason);
  }
  nesButtons = 0U;
  nesCommandDirty = true;
}

void setControlMode(ControlMode mode) {
  nesResetCommandPending = false;
  activeInputSource = INPUT_SOURCE_NONE;
  if(mode == CONTROL_MODE_NES) {
    setJoystickValues(0, 0, 0, 0);
    joystickActive = false;
    /* Entering the game controller must never leave the robot spinning from
     * a previously latched gyro command. */
    setGyroControl(false, gyroSignedSpeed);
    gyroCommandDirty = true;
    clearNesButtons("NES mode entered");
  } else {
    clearNesButtons("robot mode entered");
  }
  controlMode = mode;
  Serial.printf("[Mode] %s\n", (controlMode == CONTROL_MODE_NES) ? "NES" : "MECANUM");
}

void sendGyroCommand() {
  const int8_t direction = (gyroSignedSpeed < 0) ? -1 : 1;
  const uint8_t speedPercent = (uint8_t)abs((int)gyroSignedSpeed);
  const uint8_t txBuf[] = {
    HEADER_1, HEADER_2, 0x09, 0x0D, 0x02,
    gyroEnabled ? (uint8_t)1U : (uint8_t)0U,
    (uint8_t)direction, speedPercent, FRAME_END
  };

  Serial1.write(txBuf, sizeof(txBuf));
  gyroCommandDirty = false;
}

void sendNesCommand() {
  const uint8_t txBuf[] = {
    HEADER_1, HEADER_2, 0x08, UART_DEV_NES, 0x02,
    nesButtons, nesSequence, FRAME_END
  };

  Serial1.write(txBuf, sizeof(txBuf));
  nesCommandDirty = false;
  lastNesSendMs = millis();
}

void sendNesResetCommand() {
  const uint8_t txBuf[] = {
    HEADER_1, HEADER_2, 0x07, UART_DEV_NES, 0x03,
    nesResetSequence, FRAME_END
  };

  Serial1.write(txBuf, sizeof(txBuf));
  nesResetCommandPending = false;
}

void zeroJoystick(const char* reason) {
  const bool wasMoving = (lx != 0 || ly != 0 || rx != 0 || ry != 0);

  setJoystickValues(0, 0, 0, 0);
  joystickActive = false;
  /* Gyro is a latched command. A joystick heartbeat timeout only zeros the
   * translation command; it must never synthesize a GYRO OFF frame. */
  if(wasMoving) {
    Serial.printf("[Safety] joystick zeroed: %s\n", reason);
  }
}

const char* inputSourceName(InputSource source) {
  switch(source) {
    case INPUT_SOURCE_WEB:
      return "WEB";
    case INPUT_SOURCE_GAMEPAD:
      return "GAMEPAD";
    default:
      return "NONE";
  }
}

bool claimInputSource(InputSource source, const char* reason) {
  if(source == INPUT_SOURCE_NONE) return false;
  if(activeInputSource == source) return true;

  /* BLE has deterministic ownership while active. The continuously streamed
   * WebSocket frames therefore cannot fight an event-driven controller that
   * only reports state changes. */
  if((activeInputSource != INPUT_SOURCE_NONE) &&
     (source != INPUT_SOURCE_GAMEPAD)) {
    return false;
  }

  Serial.printf("[Input] %s -> %s: %s\n",
                inputSourceName(activeInputSource), inputSourceName(source),
                reason);
  if(activeInputSource == INPUT_SOURCE_WEB) {
    joystickActive = false;
  }
  activeInputSource = source;
  return true;
}

void releaseInputSource(InputSource source, const char* reason) {
  if(activeInputSource != source) return;

  if(controlMode == CONTROL_MODE_MECANUM) {
    zeroJoystick(reason);
  } else {
    clearNesButtons(reason);
  }
  Serial.printf("[Input] %s -> NONE: %s\n", inputSourceName(source), reason);
  activeInputSource = INPUT_SOURCE_NONE;
}

void stopJoystick(const char* reason) {
  /* Wi-Fi/WebSocket failures must never cancel a connected BLE controller. */
  releaseInputSource(INPUT_SOURCE_WEB, reason);
  activeWebSocketClient = 0xFF;
  activeWebSocketSession = 0;
  joystickSequenceValid = false;
}

uint8_t gamepadToNesButtons(const flydigi_direwolf3::State& state) {
  uint8_t buttons = 0U;
  if(state.actionPressed(flydigi_direwolf3::kButtonA)) buttons |= 0x01U;
  if(state.actionPressed(flydigi_direwolf3::kButtonB)) buttons |= 0x02U;
  if(state.miscPressed(flydigi_direwolf3::kButtonSelect)) buttons |= 0x04U;
  if(state.miscPressed(flydigi_direwolf3::kButtonStart)) buttons |= 0x08U;

  switch(state.hat) {
    case flydigi_direwolf3::kHatUp:
      buttons |= 0x10U;
      break;
    case flydigi_direwolf3::kHatUpRight:
      buttons |= 0x10U | 0x80U;
      break;
    case flydigi_direwolf3::kHatRight:
      buttons |= 0x80U;
      break;
    case flydigi_direwolf3::kHatDownRight:
      buttons |= 0x20U | 0x80U;
      break;
    case flydigi_direwolf3::kHatDown:
      buttons |= 0x20U;
      break;
    case flydigi_direwolf3::kHatDownLeft:
      buttons |= 0x20U | 0x40U;
      break;
    case flydigi_direwolf3::kHatLeft:
      buttons |= 0x40U;
      break;
    case flydigi_direwolf3::kHatUpLeft:
      buttons |= 0x10U | 0x40U;
      break;
    default:
      break;
  }
  return buttons;
}

void processGamepadInput() {
  bool connected = false;
  if(flydigi_gamepad::takeConnectionChange(&connected) && !connected) {
    releaseInputSource(INPUT_SOURCE_GAMEPAD, "BLE controller disconnected");
  }

  flydigi_direwolf3::State state;
  while(flydigi_gamepad::takeState(&state)) {
    if(controlMode == CONTROL_MODE_MECANUM) {
      const int padLx = flydigi_direwolf3::axisPercent(state.leftX);
      const int padLy = -flydigi_direwolf3::axisPercent(state.leftY);
      const int padRx = flydigi_direwolf3::axisPercent(state.rightX);
      const int padRy = -flydigi_direwolf3::axisPercent(state.rightY);
      const bool active = (padLx != 0) || (padLy != 0) ||
                          (padRx != 0) || (padRy != 0);

      if(active) {
        if(claimInputSource(INPUT_SOURCE_GAMEPAD, "BLE axes active")) {
          setJoystickValues(padLx, padLy, padRx, padRy);
        }
      } else {
        releaseInputSource(INPUT_SOURCE_GAMEPAD, "BLE axes centered");
      }
    } else {
      const uint8_t buttons = gamepadToNesButtons(state);
      if(buttons != 0U) {
        if(claimInputSource(INPUT_SOURCE_GAMEPAD, "BLE NES button active") &&
           (buttons != nesButtons)) {
          setNesButtons(buttons, gamepadNesSequence++);
          /* A short press and release can both be queued before loop() gets
           * CPU time. Transmit every transition instead of coalescing them. */
          sendNesCommand();
        }
      } else {
        if(activeInputSource == INPUT_SOURCE_GAMEPAD) {
          releaseInputSource(INPUT_SOURCE_GAMEPAD,
                             "BLE NES buttons released");
          sendNesCommand();
        }
      }
    }
  }
}

bool isWebSocketSessionRevoked(uint32_t session) {
  for(uint8_t index = 0; index < 4U; ++index) {
    if((session != 0U) && (revokedWebSocketSessions[index] == session)) {
      return true;
    }
  }
  return false;
}

void revokeWebSocketSession(uint32_t session) {
  if((session == 0U) || isWebSocketSessionRevoked(session)) return;

  revokedWebSocketSessions[revokedWebSocketWriteIndex] = session;
  revokedWebSocketWriteIndex = (uint8_t)((revokedWebSocketWriteIndex + 1U) % 4U);
}

void handleUpdate() {
  /* Reject cached legacy pages so they cannot overwrite live WebSocket
   * input belonging to the current Robot/NES controller page. */
  server.sendHeader("Cache-Control", "no-store");
  server.send(410, "text/plain", "WebSocket controller required");
}

void handleNotFound() {
  // Relative redirect works on both the router and the fallback captive portal.
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

void webSocketEvent(uint8_t client, WStype_t type, uint8_t* payload, size_t length) {
  switch(type) {
    case WStype_CONNECTED:
      Serial.printf("[WS] client %u connected\n", client);
      break;

    case WStype_DISCONNECTED:
      Serial.printf("[WS] client %u disconnected\n", client);
      if(client == activeWebSocketClient) {
        stopJoystick("websocket disconnected");
      }
      break;

    case WStype_BIN:
      if((length == 5U) && (payload[0] == WS_TYPE_HELLO)) {
        const uint32_t session = ((uint32_t)payload[1]) |
                                 ((uint32_t)payload[2] << 8) |
                                 ((uint32_t)payload[3] << 16) |
                                 ((uint32_t)payload[4] << 24);

        if((session == 0U) ||
           (isWebSocketSessionRevoked(session) &&
            (session != activeWebSocketSession))) {
          joystickRejectedCount++;
          webSocket.sendTXT(client, "TAKEN_OVER");
          break;
        }

        if((activeWebSocketClient != 0xFFU) &&
           (activeWebSocketClient != client)) {
          const uint8_t oldClient = activeWebSocketClient;
          const uint32_t oldSession = activeWebSocketSession;

          releaseInputSource(INPUT_SOURCE_WEB, "control page takeover");
          if((oldSession != 0U) && (oldSession != session)) {
            revokeWebSocketSession(oldSession);
            joystickTakeoverCount++;
          }
          activeWebSocketClient = client;
          activeWebSocketSession = session;
          joystickSequenceValid = false;
          webSocket.sendTXT(oldClient, "TAKEN_OVER");
        } else {
          activeWebSocketClient = client;
          activeWebSocketSession = session;
          joystickSequenceValid = false;
        }

        webSocket.sendTXT(client, "ACTIVE");
        Serial.printf("[WS] client %u owns control, session=%08lX\n",
                      client, (unsigned long)session);
        break;
      }

      if(client != activeWebSocketClient) {
        joystickRejectedCount++;
        break;
      }

      if((length == 2U) && (payload[0] == WS_TYPE_MODE) &&
         (payload[1] <= (uint8_t)CONTROL_MODE_NES)) {
        setControlMode((ControlMode)payload[1]);
      } else if((length == 2U) && (payload[0] == WS_TYPE_NES_RESET)) {
        if(controlMode == CONTROL_MODE_NES) {
          nesResetSequence = payload[1];
          nesResetCommandPending = true;
        }
      } else if((length == 3U) && (payload[0] == WS_TYPE_NES)) {
        if(controlMode == CONTROL_MODE_NES) {
          if(payload[1] != 0U) {
            claimInputSource(INPUT_SOURCE_WEB, "web NES button active");
          }
          if(activeInputSource == INPUT_SOURCE_WEB) {
            if(payload[1] != 0U) {
              setNesButtons(payload[1], payload[2]);
            } else {
              releaseInputSource(INPUT_SOURCE_WEB,
                                 "web NES buttons released");
            }
          }
        }
      } else if((length == 7U) && (payload[0] == WS_TYPE_AXES)) {
        const uint16_t sequence = (uint16_t)payload[1] |
                                  ((uint16_t)payload[2] << 8);
        const uint8_t ack[] = {
          WS_TYPE_ACK, (uint8_t)(sequence & 0xFFU),
          (uint8_t)((sequence >> 8) & 0xFFU)
        };

        if(controlMode == CONTROL_MODE_MECANUM) {
          const int webLx = (int8_t)payload[3];
          const int webLy = (int8_t)payload[4];
          const int webRx = (int8_t)payload[5];
          const int webRy = (int8_t)payload[6];
          const bool active = (webLx != 0) || (webLy != 0) ||
                              (webRx != 0) || (webRy != 0);
          /* Port 81 control and port 82 video still share the same Wi-Fi
           * radio.  Hold the partial JPEG briefly so this frame's ACK is not
           * trapped behind another camera TCP segment.  A held joystick gets
           * the longer guard; the centered keepalive only needs a short one. */
          cameraControlGuardUntilMs = millis() +
              (active ? CAMERA_CONTROL_ACTIVE_GUARD_MS :
                        CAMERA_CONTROL_IDLE_GUARD_MS);
          if(active) {
            claimInputSource(INPUT_SOURCE_WEB, "web axes active");
          }
          if((activeInputSource == INPUT_SOURCE_WEB) &&
             (!joystickSequenceValid || (sequence != lastJoystickSequence))) {
            setJoystickValues(webLx, webLy, webRx, webRy);
            lastJoystickSequence = sequence;
            joystickSequenceValid = true;
            joystickRxCount++;
          }
          if(activeInputSource == INPUT_SOURCE_WEB) {
            lastJoystickUpdateMs = millis();
            joystickActive = active;
            if(!active) {
              releaseInputSource(INPUT_SOURCE_WEB, "web axes centered");
            }
          }
        }
        (void)webSocket.sendBIN(client, ack, sizeof(ack));
      } else if((length == 3U) && (payload[0] == WS_TYPE_GYRO) &&
                (controlMode == CONTROL_MODE_MECANUM)) {
        setGyroControl(payload[1] != 0U, (int8_t)payload[2]);
      } else {
        joystickRejectedCount++;
      }
      break;

    default:
      break;
  }
}

void cameraWebSocketEvent(uint8_t client, WStype_t type,
                          uint8_t* payload, size_t length) {
  if(type == WStype_CONNECTED) {
    activeCameraWebSocketClient = client;
    cameraBrowserCredit = false;
    const bool noDelay = cameraWebSocket.enableLowLatency(client);
    Serial.printf("[Camera WS] client %u connected nodelay=%u\n",
                  client, noDelay ? 1U : 0U);
  } else if(type == WStype_DISCONNECTED) {
    if(client == activeCameraWebSocketClient) {
      activeCameraWebSocketClient = 0xFFU;
      cameraBrowserCredit = false;
    }
    Serial.printf("[Camera WS] client %u disconnected\n", client);
  } else if((type == WStype_BIN) &&
            (client == activeCameraWebSocketClient) &&
            (length == 1U) && (payload[0] == 0x43U)) {
    /* One browser credit permits exactly one complete JPEG. The next credit
     * is sent only after <img> has decoded the previous frame. */
    cameraBrowserCredit = true;
  }
}

void wifiEvent(WiFiEvent_t event) {
  if(event == ARDUINO_EVENT_WIFI_STA_GOT_IP) {
    staGotIp = true;
  } else if(event == ARDUINO_EVENT_WIFI_STA_DISCONNECTED) {
    staDisconnected = true;
  } else if(event == ARDUINO_EVENT_WIFI_AP_START) {
    apRunning = true;
    Serial.println("[WiFi] SoftAP started");
  } else if(event == ARDUINO_EVENT_WIFI_AP_STOP) {
    apStopped = true;
  } else if(event == ARDUINO_EVENT_WIFI_AP_STACONNECTED) {
    Serial.println("[WiFi] phone associated");
  } else if(event == ARDUINO_EVENT_WIFI_AP_STADISCONNECTED) {
    apClientDisconnected = true;
  } else if(event == ARDUINO_EVENT_WIFI_AP_STAIPASSIGNED) {
    Serial.println("[WiFi] phone received an IP address");
  }
}

uint8_t chooseBestApChannel() {
  static const uint8_t candidates[] = {1, 6, 11};
  uint16_t score[3] = {0, 0, 0};
  uint8_t best = 1; // Keep channel 6 when the scan is empty or tied.

  Serial.println("[WiFi] scanning channels before SoftAP start");
  WiFi.mode(WIFI_STA);
  delay(100);

  const int16_t count = WiFi.scanNetworks(false, true, false, 120);
  if(count > 0) {
    for(int16_t network = 0; network < count; ++network) {
      const int32_t channel = WiFi.channel(network);
      const int32_t rssi = WiFi.RSSI(network);
      uint8_t strength;

      if(rssi > -50) strength = 10;
      else if(rssi > -60) strength = 8;
      else if(rssi > -70) strength = 5;
      else if(rssi > -80) strength = 3;
      else strength = 1;

      for(uint8_t i = 0; i < 3; ++i) {
        const int32_t separation = abs(channel - candidates[i]);
        if(separation < 5) {
          score[i] += (uint16_t)(strength * (5 - separation));
        }
      }
    }

    for(uint8_t i = 0; i < 3; ++i) {
      if(score[i] < score[best]) {
        best = i;
      }
    }
  }

  WiFi.scanDelete();
  WiFi.mode(WIFI_OFF);
  delay(100);
  Serial.printf("[WiFi] channel scores 1=%u 6=%u 11=%u, selected=%u\n",
                score[0], score[1], score[2], candidates[best]);
  return candidates[best];
}

bool configureAccessPointRadio() {
  bool ok = true;
  wifi_config_t apConfig = {};
  esp_err_t countryResult;
  esp_err_t protocolResult;
  esp_err_t bandwidthResult;
  esp_err_t powerSaveResult;
  esp_err_t powerResult;
  esp_err_t getConfigResult;
  esp_err_t setConfigResult = ESP_FAIL;
  int8_t actualPower = 0;

  countryResult = esp_wifi_set_country_code("CN", true);
  protocolResult = esp_wifi_set_protocol(
      WIFI_IF_AP, WIFI_PROTOCOL_11B | WIFI_PROTOCOL_11G | WIFI_PROTOCOL_11N);
  bandwidthResult = esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20);
  /* Keep both normal and camera operation under modem sleep. Camera mode
   * pauses BLE discovery instead of increasing Wi-Fi power consumption. */
  powerSaveResult = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
  powerResult = esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);

  getConfigResult = esp_wifi_get_config(WIFI_IF_AP, &apConfig);
  if(getConfigResult == ESP_OK) {
    apConfig.ap.channel = selectedApChannel;
    apConfig.ap.authmode = WIFI_AUTH_WPA2_PSK;
    apConfig.ap.pairwise_cipher = WIFI_CIPHER_TYPE_CCMP;
    apConfig.ap.ssid_hidden = 0;
    apConfig.ap.max_connection = 4;
    apConfig.ap.beacon_interval = 100;
    setConfigResult = esp_wifi_set_config(WIFI_IF_AP, &apConfig);
  }

  (void)esp_wifi_get_max_tx_power(&actualPower);
  if((countryResult != ESP_OK) || (protocolResult != ESP_OK) ||
     (bandwidthResult != ESP_OK) || (powerSaveResult != ESP_OK) ||
     (powerResult != ESP_OK) || (getConfigResult != ESP_OK) ||
     (setConfigResult != ESP_OK)) {
    ok = false;
  }

  Serial.printf("[WiFi] radio country=%d protocol=%d bw=%d ps=%d power=%d "
                "config=%d/%d actualTx=%.2f dBm\n",
                countryResult, protocolResult, bandwidthResult,
                powerSaveResult, powerResult, getConfigResult,
                setConfigResult, (float)actualPower * 0.25f);
  return ok;
}

bool accessPointHealthy() {
  wifi_config_t apConfig = {};
  const wifi_mode_t mode = WiFi.getMode();
  const IPAddress ip = WiFi.softAPIP();

  if((mode != WIFI_AP) && (mode != WIFI_AP_STA)) return false;
  if((ip[0] != 192) || (ip[1] != 168) || (ip[2] != 4) || (ip[3] != 1)) {
    return false;
  }
  if(esp_wifi_get_config(WIFI_IF_AP, &apConfig) != ESP_OK) return false;
  /* In AP+STA mode the radio follows the router's channel automatically, so
   * the configured fallback channel must not be treated as a fault. */
  if((apConfig.ap.ssid_hidden != 0) ||
     (apConfig.ap.beacon_interval != 100)) {
    return false;
  }
  return true;
}

bool startAccessPoint() {
  const IPAddress localIp(192, 168, 4, 1);
  const IPAddress gateway(192, 168, 4, 1);
  const IPAddress subnet(255, 255, 255, 0);

  /* Keep the station interface alive so it can reconnect in the background. */
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(true);

  const bool configOk = WiFi.softAPConfig(localIp, gateway, subnet);
  const bool apOk = WiFi.softAP(fallbackApSsid, fallbackApPassword,
                                selectedApChannel, 0, 4);
  const bool radioOk = apOk ? configureAccessPointRadio() : false;

  apRunning = configOk && apOk;
  apHealthFailures = 0;
  Serial.printf("[WiFi] config=%s ap=%s radio=%s channel=%u ip=%s\n",
                configOk ? "OK" : "FAIL",
                apOk ? "OK" : "FAIL",
                radioOk ? "OK" : "WARN",
                selectedApChannel,
                WiFi.softAPIP().toString().c_str());
  if(apRunning) {
    dnsServer.stop();
    dnsServer.start(DNS_PORT, "*", localIp);
  }
  return apRunning;
}

bool startStation() {
  Serial.printf("[WiFi] connecting to %s", staSsid);
  WiFi.mode(WIFI_STA);
  const bool hostnameOk = WiFi.setHostname(mdnsHostname);
  WiFi.setSleep(true);
  WiFi.setAutoReconnect(true);
  Serial.printf(" [hostname=%s]", hostnameOk ? "OK" : "FAIL");
  WiFi.begin(staSsid, staPassword);

  const uint32_t startedAt = millis();
  while((WiFi.status() != WL_CONNECTED) &&
        (millis() - startedAt < STA_CONNECT_TIMEOUT_MS)) {
    delay(250);
    Serial.print('.');
  }
  Serial.println();

  staConnected = (WiFi.status() == WL_CONNECTED);
  staDisconnectedSinceMs = millis();
  if(staConnected) {
    Serial.printf("[WiFi] STA connected, IP=%s RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
  } else {
    Serial.println("[WiFi] STA timeout; starting recovery AP");
  }
  return staConnected;
}

void startMdnsIfNeeded() {
  if(!staConnected || mdnsRunning) return;

  lastMdnsRetryMs = millis();
  if(MDNS.begin(mdnsHostname)) {
    MDNS.setInstanceName("ESP32 Virtual Controller");
    const bool httpOk = MDNS.addService("http", "tcp", 80);
    const bool wsOk = MDNS.addService("ws", "tcp", 81);
    MDNS.enableWorkstation(ESP_IF_WIFI_STA);
    mdnsRunning = true;
    Serial.printf("[mDNS] responder=OK http=%s ws=%s url=http://%s.local\n",
                  httpOk ? "OK" : "FAIL", wsOk ? "OK" : "FAIL",
                  mdnsHostname);
  } else {
    Serial.println("[mDNS] start failed; use the STA IP printed above");
  }
}

bool prepareBleCoexistence() {
  wifi_ps_type_t powerSave = WIFI_PS_NONE;
  int8_t actualTxPower = 0;
  esp_err_t getResult = esp_wifi_get_ps(&powerSave);
  esp_err_t setResult = ESP_OK;
  const esp_err_t txSetResult =
      esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);
  const esp_err_t txGetResult = esp_wifi_get_max_tx_power(&actualTxPower);

  if((getResult == ESP_OK) && (powerSave == WIFI_PS_NONE)) {
    setResult = esp_wifi_set_ps(WIFI_PS_MIN_MODEM);
    if(setResult == ESP_OK) {
      getResult = esp_wifi_get_ps(&powerSave);
    }
  }

  const bool ready = (getResult == ESP_OK) &&
                     (setResult == ESP_OK) &&
                     (powerSave == WIFI_PS_MIN_MODEM) &&
                     (txSetResult == ESP_OK) && (txGetResult == ESP_OK);
  Serial.printf("[WiFi/BLE] coexist ps=%d get=%d set=%d tx=%d/%d "
                "power=%.2f dBm ready=%u\n",
                 (int)powerSave, (int)getResult, (int)setResult,
                 (int)txSetResult, (int)txGetResult,
                 (float)actualTxPower * 0.25f,
                 ready ? 1U : 0U);
  return ready;
}

bool setWifiHighPerformanceMode(bool enabled) {
  wifi_ps_type_t actualPowerSave = WIFI_PS_MIN_MODEM;
  int8_t actualTxPower = 0;

  /* The external 5 V rail has limited transient headroom. Keep MIN_MODEM in
   * both states and use BLE suspension alone to free airtime for camera data.
   * The enabled flag remains part of the diagnostic/session API. */
  const wifi_ps_type_t requested = WIFI_PS_MIN_MODEM;
  const esp_err_t psResult = esp_wifi_set_ps(requested);
  const esp_err_t txResult = esp_wifi_set_max_tx_power(WIFI_TX_POWER_QDBM);
  const esp_err_t getPsResult = esp_wifi_get_ps(&actualPowerSave);
  const esp_err_t getTxResult = esp_wifi_get_max_tx_power(&actualTxPower);
  const bool applied = (psResult == ESP_OK) && (getPsResult == ESP_OK) &&
                       (actualPowerSave == requested);

  Serial.printf("[WiFi Camera] request=%u applied=%u psSet=%d psGet=%d ps=%d "
                "txSet=%d txGet=%d power=%.2f dBm\n",
                enabled ? 1U : 0U, applied ? 1U : 0U,
                psResult, getPsResult,
                (int)actualPowerSave, txResult, getTxResult,
                (float)actualTxPower * 0.25f);
  return applied;
}

#if 0
void setup_legacy() {
  Serial.begin(115200); // 用于CDC调试(若需要保留)
  Serial1.begin(UART_LINK_BAUD, SERIAL_8N1, 3, 4); // 配置硬件UART1: RX=IO3, TX=IO4
  
  // Re-configure WiFi
  WiFi.disconnect(true);
  delay(100);
  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false); // Prevents C3 dropping clients
  
  IPAddress local_ip(192, 168, 4, 1);
  IPAddress gateway(192, 168, 4, 1);
  IPAddress subnet(255, 255, 255, 0);
  WiFi.softAPConfig(local_ip, gateway, subnet);
  
  // Start AP
  WiFi.softAP(ssid, password, 6, 0, 4); // Channel 6 is often more stable
  
  // Set up DNS Server for Captive Portal (Redirect all domains to 192.168.4.1)
  dnsServer.start(DNS_PORT, "*", local_ip);
  
  // Standard routes
  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_GET, handleUpdate);
  
  // Anything else redirects to Home (Captive Portal)
  server.onNotFound(handleNotFound);
  
  server.begin();
}
#endif

void setup() {
  Serial.begin(115200);
  /* 32 KiB covers about 100 ms at 2.5 Mb/s while a complete JPEG is handed
   * to the Wi-Fi stack. This prevents UART overrun without consuming the
   * memory needed by BLE and WebSocket framing. */
  const size_t cameraUartRxBufferSize = Serial1.setRxBufferSize(32768U);
  /* A 64-byte FIFO threshold leaves more hardware-FIFO headroom than the
   * Arduino default of 112 bytes. A short RX timeout still flushes partial
   * blocks promptly when the STM32 stream pauses between packets. */
  Serial1.begin(UART_LINK_BAUD, SERIAL_8N1, 3, 4, false, 20000UL, 64U);
  const bool cameraUartTimeoutSet = Serial1.setRxTimeout(2U);
  Serial1.onReceiveError(cameraUartReceiveError);

  static const uint8_t crc32TestVector[] = {
    '1', '2', '3', '4', '5', '6', '7', '8', '9'
  };
  cameraRomCrcValid =
    (esp_rom_crc32_le(0U, crc32TestVector, sizeof(crc32TestVector)) ==
     0xCBF43926UL);
  delay(200);

  Serial.printf("\n[Boot] reset reason=%d\n", (int)esp_reset_reason());
  Serial.printf("[UART1 Camera] baud=%lu rx=%u fifo=64 timeout=%u romCrc=%u\n",
                (unsigned long)Serial1.baudRate(),
                (unsigned int)cameraUartRxBufferSize,
                cameraUartTimeoutSet ? 1U : 0U,
                cameraRomCrcValid ? 1U : 0U);
  WiFi.persistent(false);
  WiFi.onEvent(wifiEvent);
  WiFi.mode(WIFI_OFF);
  delay(100);
  selectedApChannel = chooseBestApChannel();
  if(!startStation()) {
    (void)startAccessPoint();
  }

  server.on("/", HTTP_GET, handleRoot);
  server.on("/update", HTTP_GET, handleUpdate);
  server.onNotFound(handleNotFound);
  server.begin();

  webSocket.begin();
  webSocket.onEvent(webSocketEvent);
  /* Avoid disconnecting mobile browsers during brief UI/network scheduling
   * stalls. Joystick freshness is handled separately by its data watchdog. */
  webSocket.enableHeartbeat(5000, 5000, 3);
  cameraWebSocket.begin();
  cameraWebSocket.onEvent(cameraWebSocketEvent);
  /* Raw non-blocking JPEG transmission may span multiple loop iterations;
   * do not inject a heartbeat control frame into an unfinished data frame. */
  cameraWebSocket.disableHeartbeat();
  startMdnsIfNeeded();
  if(staConnected) {
    Serial.printf("[HTTP] http://%s or http://%s.local  [WS] port 81\n",
                  WiFi.localIP().toString().c_str(), mdnsHostname);
  }
  if(apRunning) {
    Serial.println("[HTTP] fallback AP: http://192.168.4.1");
  }
  gamepadBleEnabled = prepareBleCoexistence();
  if(gamepadBleEnabled) {
    flydigi_gamepad::begin();
  } else {
    Serial.println("[PAD BLE] disabled: Wi-Fi coexistence setup failed");
  }
}

void loop() {
  /* Service the latency-sensitive control socket before draining the camera
   * UART. The camera reader has a strict byte budget, so a continuous stream
   * can no longer starve joystick ACKs or HTTP/WebSocket housekeeping. */
  webSocket.loop();
  cameraWebSocket.loop();
  serviceCameraWebTx();
  if(apRunning) dnsServer.processNextRequest();
  server.handleClient();
  processCameraUart();
  if(gamepadBleEnabled) {
    flydigi_gamepad::service();
    processGamepadInput();
  }
  delay(1);

  if(apStopped) {
    apStopped = false;
    apRunning = false;
    stopJoystick("SoftAP stopped");
    Serial.println("[WiFi] SoftAP stopped");
  }

  if(staGotIp) {
    staGotIp = false;
    staDisconnected = false;
    staConnected = true;
    Serial.printf("[WiFi] STA connected, IP=%s RSSI=%d dBm\n",
                  WiFi.localIP().toString().c_str(), WiFi.RSSI());
    startMdnsIfNeeded();
  }

  if(staDisconnected) {
    staDisconnected = false;
    staConnected = false;
    staDisconnectedSinceMs = millis();
    if(mdnsRunning) {
      MDNS.end();
      mdnsRunning = false;
    }
    Serial.println("[WiFi] STA disconnected; reconnecting");
  }

  if(apClientDisconnected) {
    apClientDisconnected = false;
    /* Do not stop a controller connected through the router merely because a
     * recovery-AP client left. The WebSocket callback identifies the owner. */
    Serial.println("[WiFi] station disconnected");
  }

  const uint32_t now = millis();
  serviceCameraStreamRequest(now);
  if((activeInputSource == INPUT_SOURCE_WEB) && joystickActive &&
     (now - lastJoystickUpdateMs >= JOYSTICK_TIMEOUT_MS)) {
    /* Retain page ownership so its next sequenced frame resumes immediately. */
    releaseInputSource(INPUT_SOURCE_WEB, "control heartbeat timeout");
  }
  if((activeInputSource == INPUT_SOURCE_WEB) &&
     (controlMode == CONTROL_MODE_NES) && (nesButtons != 0U) &&
     (now - lastNesUpdateMs >= NES_INPUT_TIMEOUT_MS)) {
    releaseInputSource(INPUT_SOURCE_WEB, "NES heartbeat timeout");
  }

  if(!staConnected && (now - lastStaRetryMs >= STA_RETRY_INTERVAL_MS)) {
    lastStaRetryMs = now;
    if(WiFi.status() != WL_CONNECTED) {
      Serial.println("[WiFi] retrying router connection");
      (void)WiFi.reconnect();
    }
  }

  if(staConnected && !mdnsRunning &&
     (now - lastMdnsRetryMs >= MDNS_RETRY_INTERVAL_MS)) {
    startMdnsIfNeeded();
  }

  if(!staConnected && !apRunning &&
     (now - staDisconnectedSinceMs >= STA_FALLBACK_DELAY_MS)) {
    Serial.println("[WiFi] router unavailable; enabling recovery AP");
    lastApRetryMs = now;
    (void)startAccessPoint();
  }

  if(!apRunning && (WiFi.getMode() == WIFI_AP_STA) &&
     (now - lastApRetryMs >= AP_RETRY_INTERVAL_MS)) {
    lastApRetryMs = now;
    Serial.println("[WiFi] retrying SoftAP start");
    (void)startAccessPoint();
  }

  if(apRunning && (now - lastApHealthMs >= AP_HEALTH_INTERVAL_MS)) {
    lastApHealthMs = now;
    if(accessPointHealthy()) {
      apHealthFailures = 0;
    } else {
      if(apHealthFailures < 0xFF) apHealthFailures++;
      Serial.printf("[WiFi] AP health check failed (%u/2)\n", apHealthFailures);
      if(apHealthFailures >= 2) {
        stopJoystick("SoftAP health failure");
        apRunning = false;
        /* Keep the STA radio and its router reconnection alive. */
        (void)WiFi.softAPdisconnect(false);
        lastApRetryMs = now;
      }
    }
  }

  if(now - lastStatusMs >= STATUS_INTERVAL_MS) {
    lastStatusMs = now;
    Serial.printf("[Status] STA=%u ip=%s rssi=%d AP=%u clients=%u heap=%u mode=%s "
                  "source=%s axes=%d,%d,%d,%d gyro=%u/%d nes=%02X ws=%u "
                  "session=%08lX rx=%lu reject=%lu takeover=%lu "
                  "pad=%u/%d reports=%lu/%lu\n",
                  staConnected ? 1U : 0U,
                  staConnected ? WiFi.localIP().toString().c_str() : "0.0.0.0",
                  staConnected ? WiFi.RSSI() : 0,
                  apRunning ? 1U : 0U,
                  WiFi.softAPgetStationNum(),
                  ESP.getFreeHeap(),
                  (controlMode == CONTROL_MODE_NES) ? "NES" : "ROBOT",
                  inputSourceName(activeInputSource),
                  lx, ly, rx, ry,
                  gyroEnabled ? 1U : 0U, gyroSignedSpeed, nesButtons,
                  activeWebSocketClient, (unsigned long)activeWebSocketSession,
                  (unsigned long)joystickRxCount,
                  (unsigned long)joystickRejectedCount,
                  (unsigned long)joystickTakeoverCount,
                  flydigi_gamepad::isConnected() ? 1U : 0U,
                  flydigi_gamepad::rssi(),
                  (unsigned long)flydigi_gamepad::reportCount(),
                  (unsigned long)flydigi_gamepad::droppedReportCount());
    Serial.printf("[Camera] request=%u turbo=%u ws=%u packets=%lu crc=%lu format=%lu "
                  "jpeg=%lu asmDrop=%lu web=%lu throttle=%lu wsDrop=%lu age=%lums\n",
                  cameraStreamRequested ? 1U : 0U,
                  wifiCameraTurbo ? 1U : 0U,
                  activeCameraWebSocketClient,
                  (unsigned long)cameraUartPacketCount,
                  (unsigned long)cameraUartCrcErrorCount,
                  (unsigned long)cameraUartFormatErrorCount,
                  (unsigned long)cameraJpegCompletedCount,
                  (unsigned long)cameraJpegAssemblyDropCount,
                  (unsigned long)cameraJpegWebSentCount,
                  (unsigned long)cameraJpegThrottleDropCount,
                  (unsigned long)cameraWebSocketDropCount,
                  (unsigned long)(cameraLastPacketMs ?
                    (now - cameraLastPacketMs) : 0U));
    Serial.printf("[Camera UART] bytes=%lu reads=%lu backlog=%lu/%d "
                  "err=F%lu B%lu R%lu P%lu K%lu last=%u\n",
                  (unsigned long)cameraUartBytesRead,
                  (unsigned long)cameraUartBulkReadCount,
                  (unsigned long)cameraUartMaxBacklog,
                  Serial1.available(),
                  (unsigned long)cameraUartFifoOverflowCount,
                  (unsigned long)cameraUartBufferFullCount,
                  (unsigned long)cameraUartFrameErrorCount,
                  (unsigned long)cameraUartParityErrorCount,
                  (unsigned long)cameraUartBreakErrorCount,
                  (unsigned int)cameraUartLastDriverError);
    Serial.printf("[Camera Pipe] start=%lu overlap=%lu skip=T%lu R%lu "
                  "drop=O%lu J%lu last=S%u %lu/%lu "
                  "tx=%u %lu/%lu last=%lums max=%lums "
                  "wait=%lu partial=%lu credit=%u\n",
                  (unsigned long)cameraJpegStartCount,
                  (unsigned long)cameraJpegStartOverlapCount,
                  (unsigned long)cameraJpegStartTxSuppressedCount,
                  (unsigned long)cameraJpegStartRateSuppressedCount,
                  (unsigned long)cameraJpegOrderDropCount,
                  (unsigned long)cameraJpegMarkerDropCount,
                  (unsigned int)cameraJpegLastDropSequence,
                  (unsigned long)cameraJpegLastReceivedOffset,
                  (unsigned long)cameraJpegLastExpectedOffset,
                  cameraWebTxActive ? 1U : 0U,
                  (unsigned long)cameraWebTxOffset,
                  (unsigned long)cameraWebTxSize,
                  (unsigned long)cameraWebTxLastDurationMs,
                  (unsigned long)cameraWebTxMaxDurationMs,
                  (unsigned long)cameraWebTxWouldBlockCount,
                  (unsigned long)cameraWebTxPartialWriteCount,
                  cameraBrowserCredit ? 1U : 0U);
    Serial.printf("[Camera Flow] ack=%u seq=%u age=%lums defer=%lu release=%lu\n",
                  cameraFrameAckPending ? 1U : 0U,
                  (unsigned int)cameraFrameAckSequence,
                  (unsigned long)(cameraFrameAckPending ?
                    (now - cameraFrameAckStartedMs) : 0U),
                  (unsigned long)cameraFrameAckDeferredCount,
                  (unsigned long)cameraFrameAckReleasedCount);
  }

  if ((controlMode == CONTROL_MODE_MECANUM) &&
      (now - lastSendTime >= sendIntervalMs)) {
    lastSendTime = now;
    
    uint8_t frameLen = 0x0A;
    uint8_t m1_val = (uint8_t)(int8_t)lx;
    uint8_t m2_val = (uint8_t)(int8_t)ly;
    uint8_t m3_val = (uint8_t)(int8_t)rx;
    uint8_t m4_val = (uint8_t)(int8_t)ry;

    const uint8_t CMD_BYTE = 0x0C;
    uint8_t checksum = 0; 
    
    uint8_t tx_buf[] = {
      HEADER_1, 
      HEADER_2, 
      frameLen, 
      CMD_BYTE,
      m1_val,
      m2_val,
      m3_val,
      m4_val,
      checksum,
      FRAME_END
    };

    Serial1.write(tx_buf, sizeof(tx_buf)); // 发送到硬件UART
  }

  if(nesCommandDirty ||
     ((controlMode == CONTROL_MODE_NES) &&
      (now - lastNesSendMs >= NES_KEEPALIVE_MS))) {
    sendNesCommand();
  }
  if(nesResetCommandPending) {
    sendNesResetCommand();
  }

  /* ON/OFF is event driven and latched by STM32. No periodic gyro heartbeat:
   * if no new command arrives, neither side changes the current mode. */
  if(gyroCommandDirty) {
    sendGyroCommand();
  }
  processCameraUart();
}
