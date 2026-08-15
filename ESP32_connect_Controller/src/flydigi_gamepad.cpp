#include "flydigi_gamepad.h"

#include <Arduino.h>
#include <NimBLEDevice.h>

#include <string>
#include <vector>

namespace flydigi_gamepad {
namespace {

constexpr char kTargetAddress[] = "a4:c1:38:34:2f:4f";
constexpr uint32_t kScanSeconds = 5U;
constexpr uint32_t kRetryDelayMs = 3000U;
constexpr uint8_t kStateQueueCapacity = 16U;

NimBLEAdvertisedDevice* gAdvertisedDevice = nullptr;
NimBLEClient* gClient = nullptr;
volatile bool gConnectRequested = false;
volatile bool gDisconnectPending = false;
bool gScanRunning = false;
bool gReady = false;
uint32_t gNextScanAt = 0;
uint16_t gSubscribedReports = 0;

portMUX_TYPE gEventMux = portMUX_INITIALIZER_UNLOCKED;
flydigi_direwolf3::State gStateQueue[kStateQueueCapacity];
uint8_t gStateQueueHead = 0U;
uint8_t gStateQueueTail = 0U;
uint8_t gStateQueueCount = 0U;
bool gConnectionPending = false;
bool gPendingConnected = false;
uint32_t gReports = 0;
uint32_t gDroppedReports = 0;

void publishConnection(bool connected) {
  portENTER_CRITICAL(&gEventMux);
  if(!connected) {
    /* A pre-disconnect axis frame must never re-latch motion afterwards. */
    gStateQueueHead = 0U;
    gStateQueueTail = 0U;
    gStateQueueCount = 0U;
  }
  gPendingConnected = connected;
  gConnectionPending = true;
  portEXIT_CRITICAL(&gEventMux);
}

void publishState(const flydigi_direwolf3::State& state) {
  portENTER_CRITICAL(&gEventMux);
  if(gStateQueueCount == kStateQueueCapacity) {
    /* Retain the newest physical state if the Arduino loop was briefly busy. */
    gStateQueueTail = (uint8_t)((gStateQueueTail + 1U) % kStateQueueCapacity);
    --gStateQueueCount;
    ++gDroppedReports;
  }
  gStateQueue[gStateQueueHead] = state;
  gStateQueueHead = (uint8_t)((gStateQueueHead + 1U) % kStateQueueCapacity);
  ++gStateQueueCount;
  ++gReports;
  portEXIT_CRITICAL(&gEventMux);
}

void reportCallback(NimBLERemoteCharacteristic* characteristic, uint8_t* data,
                    size_t length, bool isNotify) {
  (void)characteristic;
  (void)isNotify;
  flydigi_direwolf3::State state;
  if (flydigi_direwolf3::parseInputReport(data, length, &state)) {
    publishState(state);
  }
}

class ClientCallbacks : public NimBLEClientCallbacks {
 public:
  void onConnect(NimBLEClient* client) override {
    Serial.printf("[PAD BLE] link connected to %s\n",
                  client->getPeerAddress().toString().c_str());
  }

  void onDisconnect(NimBLEClient* client) override {
    Serial.printf("[PAD BLE] link disconnected, error=%d\n",
                  client->getLastError());
    if (gReady) {
      gReady = false;
      publishConnection(false);
    }
    gDisconnectPending = true;
  }

  uint32_t onPassKeyRequest() override {
    return 0U;
  }

  bool onConfirmPIN(uint32_t pin) override {
    Serial.printf("[PAD BLE] pairing confirmation %06lu accepted\n",
                  static_cast<unsigned long>(pin));
    return true;
  }

  void onAuthenticationComplete(ble_gap_conn_desc* descriptor) override {
    Serial.printf("[PAD BLE] security encrypted=%u bonded=%u key=%u\n",
                  descriptor->sec_state.encrypted,
                  descriptor->sec_state.bonded,
                  descriptor->sec_state.key_size);
  }
};

class AdvertisementCallbacks : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* device) override {
    if (gConnectRequested ||
        (device->getAddress().toString() != kTargetAddress)) {
      return;
    }

    Serial.printf("[PAD BLE] found %s RSSI=%d, connecting\n",
                  device->getAddress().toString().c_str(), device->getRSSI());
    gAdvertisedDevice = device;
    gConnectRequested = true;
    gScanRunning = false;
    NimBLEDevice::getScan()->stop();
  }
};

ClientCallbacks gClientCallbacks;
AdvertisementCallbacks gAdvertisementCallbacks;

void scanCompleteCallback(NimBLEScanResults results) {
  gScanRunning = false;
  if (!gConnectRequested) {
    Serial.printf("[PAD BLE] scan complete (%d devices), FN+A not found\n",
                  results.getCount());
    gNextScanAt = millis() + kRetryDelayMs;
  }
}

void deleteDisconnectedClient() {
  if ((gClient != nullptr) && !gClient->isConnected()) {
    NimBLEDevice::deleteClient(gClient);
    gClient = nullptr;
  }
}

void startScan() {
  if (gScanRunning || gConnectRequested ||
      ((gClient != nullptr) && gClient->isConnected())) {
    return;
  }

  NimBLEScan* scan = NimBLEDevice::getScan();
  scan->clearResults();
  scan->setAdvertisedDeviceCallbacks(&gAdvertisementCallbacks, false);
  /* The MAC address is known, so passive low-duty scanning is enough and
   * leaves substantially more 2.4-GHz airtime for the Wi-Fi controller. */
  scan->setActiveScan(false);
  scan->setInterval(160);
  scan->setWindow(16);

  gAdvertisedDevice = nullptr;
  gScanRunning = true;
  Serial.println("[PAD BLE] scanning for Direwolf 3 FN+A");
  if (!scan->start(kScanSeconds, scanCompleteCallback, false)) {
    gScanRunning = false;
    gNextScanAt = millis() + kRetryDelayMs;
  }
}

bool subscribeInputReports(NimBLEClient* client) {
  NimBLERemoteService* hidService = client->getService("1812");
  if (hidService == nullptr) {
    Serial.println("[PAD BLE] HID service 0x1812 not found");
    return false;
  }

  std::vector<NimBLERemoteCharacteristic*>* characteristics =
      hidService->getCharacteristics(true);
  if (characteristics == nullptr) {
    Serial.println("[PAD BLE] HID characteristic discovery failed");
    return false;
  }

  gSubscribedReports = 0U;
  const NimBLEUUID reportUuid("2a4d");
  for (NimBLERemoteCharacteristic* characteristic : *characteristics) {
    if (!characteristic->getUUID().equals(reportUuid)) {
      continue;
    }

    bool subscribed = false;
    if (characteristic->canNotify()) {
      subscribed = characteristic->subscribe(true, reportCallback, true);
    } else if (characteristic->canIndicate()) {
      subscribed = characteristic->subscribe(false, reportCallback, true);
    }
    if (subscribed) {
      ++gSubscribedReports;
    }
  }

  Serial.printf("[PAD BLE] HID report subscriptions=%u\n", gSubscribedReports);
  return gSubscribedReports != 0U;
}

bool connectTarget() {
  if (gAdvertisedDevice == nullptr) {
    return false;
  }

  deleteDisconnectedClient();
  gClient = NimBLEDevice::createClient();
  if (gClient == nullptr) {
    return false;
  }

  gClient->setClientCallbacks(&gClientCallbacks, false);
  // Favor BLE responsiveness without monopolizing the shared Wi-Fi radio.
  gClient->setConnectionParams(12, 24, 0, 200);
  gClient->setConnectTimeout(8);

  if (!gClient->connect(gAdvertisedDevice, true)) {
    Serial.printf("[PAD BLE] connection failed, error=%d\n",
                  gClient->getLastError());
    deleteDisconnectedClient();
    return false;
  }

  const bool secured = gClient->secureConnection();
  Serial.printf("[PAD BLE] security=%s RSSI=%d\n",
                secured ? "OK" : "NOT-REQUIRED/FAILED", gClient->getRssi());
  if (!gClient->isConnected() || !subscribeInputReports(gClient)) {
    if (gClient->isConnected()) {
      gClient->disconnect();
    }
    return false;
  }

  NimBLEDevice::getScan()->clearResults();
  gAdvertisedDevice = nullptr;
  gReady = true;
  publishConnection(true);
  Serial.println("[PAD BLE] Direwolf 3 ready");
  return true;
}

}  // namespace

void begin() {
  NimBLEDevice::init("STM32-KIT Controller");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityAuth(true, false, false);
  gNextScanAt = millis();
  Serial.printf("[PAD BLE] initialized, local=%s bonds=%d\n",
                NimBLEDevice::getAddress().toString().c_str(),
                NimBLEDevice::getNumBonds());
}

void service() {
  if (gDisconnectPending) {
    gDisconnectPending = false;
    deleteDisconnectedClient();
    gNextScanAt = millis() + kRetryDelayMs;
  }

  if (gConnectRequested) {
    gConnectRequested = false;
    if (!connectTarget()) {
      gNextScanAt = millis() + kRetryDelayMs;
    }
  }

  const uint32_t now = millis();
  if (!gReady && !gScanRunning && !gConnectRequested &&
      (static_cast<int32_t>(now - gNextScanAt) >= 0)) {
    startScan();
  }
}

bool takeState(flydigi_direwolf3::State* state) {
  if (state == nullptr) {
    return false;
  }

  bool available = false;
  portENTER_CRITICAL(&gEventMux);
  if(gStateQueueCount != 0U) {
    *state = gStateQueue[gStateQueueTail];
    gStateQueueTail = (uint8_t)((gStateQueueTail + 1U) % kStateQueueCapacity);
    --gStateQueueCount;
    available = true;
  }
  portEXIT_CRITICAL(&gEventMux);
  return available;
}

bool takeConnectionChange(bool* connected) {
  if (connected == nullptr) {
    return false;
  }

  bool available = false;
  portENTER_CRITICAL(&gEventMux);
  if (gConnectionPending) {
    *connected = gPendingConnected;
    gConnectionPending = false;
    available = true;
  }
  portEXIT_CRITICAL(&gEventMux);
  return available;
}

bool isConnected() {
  return gReady;
}

int rssi() {
  if ((gClient == nullptr) || !gClient->isConnected()) {
    return 0;
  }
  return gClient->getRssi();
}

uint32_t reportCount() {
  uint32_t reports;
  portENTER_CRITICAL(&gEventMux);
  reports = gReports;
  portEXIT_CRITICAL(&gEventMux);
  return reports;
}

uint32_t droppedReportCount() {
  uint32_t reports;
  portENTER_CRITICAL(&gEventMux);
  reports = gDroppedReports;
  portEXIT_CRITICAL(&gEventMux);
  return reports;
}

}  // namespace flydigi_gamepad
