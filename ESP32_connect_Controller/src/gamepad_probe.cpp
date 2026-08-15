#include <Arduino.h>
#include <NimBLEDevice.h>

#include "flydigi_direwolf3_report.h"

#include <cstring>
#include <string>
#include <vector>

namespace {

constexpr char kTargetName[] = "Flydigi Direwolf 3";
constexpr char kDInputAddress[] = "a4:c1:38:34:2f:4f";
constexpr char kFlashplayAddress[] = "a4:c1:38:34:2f:50";

constexpr uint32_t kScanSeconds = 8;
constexpr uint32_t kRetryDelayMs = 1500;
constexpr uint32_t kStatusIntervalMs = 5000;
constexpr size_t kMaxHexBytes = 512;
constexpr size_t kTrackedReports = 16;
constexpr size_t kReportSnapshotBytes = 128;

struct ReportSnapshot {
  uint16_t handle = 0;
  uint16_t length = 0;
  bool valid = false;
  uint8_t data[kReportSnapshotBytes] = {};
};

NimBLEAdvertisedDevice* gAdvertisedDevice = nullptr;
NimBLEClient* gClient = nullptr;
volatile bool gConnectRequested = false;
volatile bool gDisconnected = false;
bool gScanRunning = false;
uint32_t gNextScanAt = 0;
uint32_t gLastStatusAt = 0;
uint32_t gNotificationCount = 0;
uint32_t gDuplicateCount = 0;
uint16_t gSubscriptionCount = 0;
ReportSnapshot gSnapshots[kTrackedReports];

void printButtonName(bool pressed, const char* name, bool* any) {
  if (!pressed) {
    return;
  }
  Serial.printf("%s%s", *any ? "," : "", name);
  *any = true;
}

void printDecodedReport(const uint8_t* data, size_t length) {
  flydigi_direwolf3::State state;
  if (!flydigi_direwolf3::parseInputReport(data, length, &state)) {
    return;
  }

  Serial.printf("[PAD %10lu] LX=%4d LY=%4d RX=%4d RY=%4d "
                "HAT=%-10s LT=%3u RT=%3u buttons=",
                static_cast<unsigned long>(millis()),
                flydigi_direwolf3::axisPercent(state.leftX),
                flydigi_direwolf3::axisPercent(state.leftY),
                flydigi_direwolf3::axisPercent(state.rightX),
                flydigi_direwolf3::axisPercent(state.rightY),
                flydigi_direwolf3::hatName(state.hat), state.leftTrigger,
                state.rightTrigger);

  bool any = false;
  printButtonName(state.actionPressed(flydigi_direwolf3::kButtonA), "A", &any);
  printButtonName(state.actionPressed(flydigi_direwolf3::kButtonB), "B", &any);
  printButtonName(state.actionPressed(flydigi_direwolf3::kButtonX), "X", &any);
  printButtonName(state.actionPressed(flydigi_direwolf3::kButtonY), "Y", &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonLB), "LB", &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonRB), "RB", &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonLT), "LT", &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonRT), "RT", &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonSelect), "SELECT",
                  &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonStart), "START",
                  &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonLeftStick), "LS",
                  &any);
  printButtonName(state.miscPressed(flydigi_direwolf3::kButtonRightStick), "RS",
                  &any);
  if (!any) {
    Serial.print('-');
  }

  Serial.printf(" gyro=%d,%d,%d rawAxes=%02X,%02X,%02X,%02X\n",
                state.gyroX, state.gyroY, state.gyroZ,
                static_cast<uint8_t>(state.leftX),
                static_cast<uint8_t>(state.leftY),
                static_cast<uint8_t>(state.rightX),
                static_cast<uint8_t>(state.rightY));
}

void printHex(const uint8_t* data, size_t length, size_t limit = kMaxHexBytes) {
  if ((data == nullptr) || (length == 0U)) {
    Serial.print("<empty>");
    return;
  }

  const size_t printed = (length < limit) ? length : limit;
  for (size_t index = 0; index < printed; ++index) {
    if (index != 0U) {
      Serial.print(' ');
    }
    Serial.printf("%02X", data[index]);
  }
  if (printed < length) {
    Serial.printf(" ...(+%u bytes)", static_cast<unsigned>(length - printed));
  }
}

void printStringHex(const std::string& value) {
  printHex(reinterpret_cast<const uint8_t*>(value.data()), value.size());
}

const char* modeForAddress(const std::string& address) {
  if (address == kDInputAddress) {
    return "FN+A DInput";
  }
  if (address == kFlashplayAddress) {
    return "FN+Y Flashplay";
  }
  return "name match / unknown mode";
}

bool isTarget(NimBLEAdvertisedDevice* device) {
  const std::string address = device->getAddress().toString();
  if ((address == kDInputAddress) || (address == kFlashplayAddress)) {
    return true;
  }
  return device->haveName() && (device->getName() == kTargetName);
}

void printAdvertisement(NimBLEAdvertisedDevice* device) {
  const std::string address = device->getAddress().toString();
  Serial.println();
  Serial.println("========== TARGET ADVERTISEMENT ==========");
  Serial.printf("name       : %s\n",
                device->haveName() ? device->getName().c_str() : "<none>");
  Serial.printf("address    : %s (type=%u)\n", address.c_str(),
                static_cast<unsigned>(device->getAddressType()));
  Serial.printf("mode guess : %s\n", modeForAddress(address));
  Serial.printf("RSSI       : %d dBm\n", device->getRSSI());
  Serial.printf("appearance : 0x%04X\n", device->getAppearance());

  Serial.printf("mfg data   : ");
  if (device->haveManufacturerData()) {
    printStringHex(device->getManufacturerData());
  } else {
    Serial.print("<none>");
  }
  Serial.println();

  const uint8_t serviceCount = device->getServiceUUIDCount();
  Serial.printf("adv services (%u):", static_cast<unsigned>(serviceCount));
  for (uint8_t index = 0; index < serviceCount; ++index) {
    Serial.printf(" %s", device->getServiceUUID(index).toString().c_str());
  }
  if (serviceCount == 0U) {
    Serial.print(" <none>");
  }
  Serial.println();
  Serial.println("==========================================");
}

ReportSnapshot* snapshotForHandle(uint16_t handle) {
  ReportSnapshot* freeSlot = nullptr;
  for (auto& snapshot : gSnapshots) {
    if (snapshot.valid && (snapshot.handle == handle)) {
      return &snapshot;
    }
    if (!snapshot.valid && (freeSlot == nullptr)) {
      freeSlot = &snapshot;
    }
  }

  if (freeSlot != nullptr) {
    freeSlot->handle = handle;
    freeSlot->valid = true;
  }
  return freeSlot;
}

bool reportChanged(ReportSnapshot* snapshot, const uint8_t* data, size_t length) {
  if (snapshot == nullptr) {
    return true;
  }

  const size_t storedLength =
      (length < kReportSnapshotBytes) ? length : kReportSnapshotBytes;
  const bool changed =
      (snapshot->length != storedLength) ||
      (std::memcmp(snapshot->data, data, storedLength) != 0);
  if (changed) {
    std::memcpy(snapshot->data, data, storedLength);
    snapshot->length = static_cast<uint16_t>(storedLength);
  }
  return changed;
}

void printChangedBytes(const ReportSnapshot* previous, const uint8_t* data,
                       size_t length) {
  if ((previous == nullptr) || (previous->length == 0U)) {
    return;
  }

  const size_t compared =
      (length < previous->length) ? length : previous->length;
  bool any = false;
  Serial.print(" delta=");
  for (size_t index = 0; index < compared; ++index) {
    if (previous->data[index] != data[index]) {
      Serial.printf("%s%u:%02X>%02X", any ? "," : "",
                    static_cast<unsigned>(index), previous->data[index],
                    data[index]);
      any = true;
    }
  }
  if (!any && (length != previous->length)) {
    Serial.printf("len:%u>%u", static_cast<unsigned>(previous->length),
                  static_cast<unsigned>(length));
  } else if (!any) {
    Serial.print("none");
  }
}

void notificationCallback(NimBLERemoteCharacteristic* characteristic,
                          uint8_t* data, size_t length, bool isNotify) {
  ++gNotificationCount;
  const uint16_t handle = characteristic->getHandle();
  ReportSnapshot* snapshot = snapshotForHandle(handle);

  // Many controllers send a periodic unchanged state. Suppressing exact
  // duplicates keeps the serial log usable without losing state changes.
  bool changed = true;
  ReportSnapshot previous;
  if (snapshot != nullptr) {
    previous = *snapshot;
    changed = reportChanged(snapshot, data, length);
  }
  if (!changed) {
    ++gDuplicateCount;
    return;
  }

  Serial.printf("[RX %10lu] %s svc=%s chr=%s handle=0x%04X len=%u data=",
                static_cast<unsigned long>(millis()),
                isNotify ? "NOTIFY" : "INDICATE",
                characteristic->getRemoteService()->getUUID().toString().c_str(),
                characteristic->getUUID().toString().c_str(), handle,
                static_cast<unsigned>(length));
  printHex(data, length, kReportSnapshotBytes);
  printChangedBytes(previous.valid ? &previous : nullptr, data, length);
  Serial.println();

  const NimBLEUUID serviceUuid =
      characteristic->getRemoteService()->getUUID();
  const NimBLEUUID characteristicUuid = characteristic->getUUID();
  if (serviceUuid.equals(NimBLEUUID("1812")) &&
      characteristicUuid.equals(NimBLEUUID("2a4d")) &&
      (length == flydigi_direwolf3::kInputReportLength)) {
    printDecodedReport(data, length);
  }
}

class ProbeClientCallbacks : public NimBLEClientCallbacks {
 public:
  void onConnect(NimBLEClient* client) override {
    Serial.printf("[BLE] connected to %s, MTU=%u\n",
                  client->getPeerAddress().toString().c_str(), client->getMTU());
  }

  void onDisconnect(NimBLEClient* client) override {
    Serial.printf("[BLE] disconnected from %s, error=%d\n",
                  client->getPeerAddress().toString().c_str(),
                  client->getLastError());
    gDisconnected = true;
  }

  uint32_t onPassKeyRequest() override {
    Serial.println("[SEC] passkey requested; replying 000000");
    return 0;
  }

  bool onConfirmPIN(uint32_t pin) override {
    Serial.printf("[SEC] numeric comparison %06lu accepted\n",
                  static_cast<unsigned long>(pin));
    return true;
  }

  void onAuthenticationComplete(ble_gap_conn_desc* descriptor) override {
    Serial.printf("[SEC] encrypted=%u authenticated=%u bonded=%u key=%u\n",
                  descriptor->sec_state.encrypted,
                  descriptor->sec_state.authenticated,
                  descriptor->sec_state.bonded,
                  descriptor->sec_state.key_size);
  }
};

class ProbeAdvertisementCallbacks : public NimBLEAdvertisedDeviceCallbacks {
 public:
  void onResult(NimBLEAdvertisedDevice* device) override {
    if (gConnectRequested || !isTarget(device)) {
      return;
    }

    printAdvertisement(device);
    gAdvertisedDevice = device;
    gConnectRequested = true;
    gScanRunning = false;
    NimBLEDevice::getScan()->stop();
  }
};

ProbeClientCallbacks gClientCallbacks;
ProbeAdvertisementCallbacks gAdvertisementCallbacks;

void scanCompleteCallback(NimBLEScanResults results) {
  gScanRunning = false;
  if (!gConnectRequested) {
    Serial.printf("[SCAN] complete, %d devices; target not found\n",
                  results.getCount());
    gNextScanAt = millis() + kRetryDelayMs;
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
  scan->setActiveScan(true);
  scan->setInterval(80);
  scan->setWindow(48);

  gAdvertisedDevice = nullptr;
  gScanRunning = true;
  Serial.printf("[SCAN] looking for %s for %lu seconds...\n", kTargetName,
                static_cast<unsigned long>(kScanSeconds));
  if (!scan->start(kScanSeconds, scanCompleteCallback, false)) {
    gScanRunning = false;
    gNextScanAt = millis() + kRetryDelayMs;
    Serial.println("[SCAN] failed to start");
  }
}

void printProperties(NimBLERemoteCharacteristic* characteristic) {
  Serial.printf("%c%c%c%c%c%c",
                characteristic->canRead() ? 'R' : '-',
                characteristic->canWrite() ? 'W' : '-',
                characteristic->canWriteNoResponse() ? 'w' : '-',
                characteristic->canNotify() ? 'N' : '-',
                characteristic->canIndicate() ? 'I' : '-',
                characteristic->canBroadcast() ? 'B' : '-');
}

void dumpDescriptors(NimBLERemoteCharacteristic* characteristic) {
  std::vector<NimBLERemoteDescriptor*>* descriptors =
      characteristic->getDescriptors(true);
  if (descriptors == nullptr) {
    Serial.println("      descriptors: discovery failed");
    return;
  }

  for (NimBLERemoteDescriptor* descriptor : *descriptors) {
    Serial.printf("      DESC uuid=%s handle=0x%04X value=",
                  descriptor->getUUID().toString().c_str(),
                  descriptor->getHandle());
    const NimBLEAttValue value = descriptor->readValue();
    printHex(value.data(), value.size());
    Serial.println();
  }
}

void dumpGattDatabase(NimBLEClient* client) {
  Serial.println();
  Serial.println("================ GATT DATABASE ================");
  std::vector<NimBLERemoteService*>* services = client->getServices(true);
  if (services == nullptr) {
    Serial.println("[GATT] service discovery failed");
    Serial.println("================================================");
    return;
  }

  Serial.printf("[GATT] %u services discovered\n",
                static_cast<unsigned>(services->size()));
  gSubscriptionCount = 0;

  for (NimBLERemoteService* service : *services) {
    Serial.printf("  SERVICE %s\n", service->getUUID().toString().c_str());
    std::vector<NimBLERemoteCharacteristic*>* characteristics =
        service->getCharacteristics(true);
    if (characteristics == nullptr) {
      Serial.println("    characteristic discovery failed");
      continue;
    }

    for (NimBLERemoteCharacteristic* characteristic : *characteristics) {
      Serial.printf("    CHAR uuid=%s handle=0x%04X props=",
                    characteristic->getUUID().toString().c_str(),
                    characteristic->getHandle());
      printProperties(characteristic);
      Serial.println();

      if (characteristic->canRead()) {
        const NimBLEAttValue value = characteristic->readValue();
        Serial.printf("      READ len=%u value=",
                      static_cast<unsigned>(value.size()));
        printHex(value.data(), value.size());
        Serial.println();
      }

      dumpDescriptors(characteristic);

      bool subscribed = false;
      if (characteristic->canNotify()) {
        subscribed =
            characteristic->subscribe(true, notificationCallback, true);
      } else if (characteristic->canIndicate()) {
        subscribed =
            characteristic->subscribe(false, notificationCallback, true);
      }

      if (characteristic->canNotify() || characteristic->canIndicate()) {
        Serial.printf("      SUBSCRIBE %s\n", subscribed ? "OK" : "FAILED");
        if (subscribed) {
          ++gSubscriptionCount;
        }
      }
    }
  }

  Serial.printf("[GATT] subscriptions=%u\n", gSubscriptionCount);
  Serial.println("================================================");
  Serial.println("Operate one control at a time and copy the [RX] lines.");
  Serial.println();
}

void discardClient() {
  if (gClient == nullptr) {
    return;
  }
  if (gClient->isConnected()) {
    gClient->disconnect();
  }
  NimBLEDevice::deleteClient(gClient);
  gClient = nullptr;
}

bool connectTarget() {
  if (gAdvertisedDevice == nullptr) {
    Serial.println("[BLE] target pointer missing; rescanning");
    return false;
  }

  discardClient();
  gClient = NimBLEDevice::createClient();
  if (gClient == nullptr) {
    Serial.println("[BLE] createClient failed");
    return false;
  }

  gClient->setClientCallbacks(&gClientCallbacks, false);
  // 7.5-15 ms connection interval, no slave latency, 2 s supervision timeout.
  gClient->setConnectionParams(6, 12, 0, 200);
  gClient->setConnectTimeout(10);

  Serial.printf("[BLE] connecting to %s...\n",
                gAdvertisedDevice->getAddress().toString().c_str());
  if (!gClient->connect(gAdvertisedDevice, true)) {
    Serial.printf("[BLE] connection failed, error=%d\n", gClient->getLastError());
    discardClient();
    return false;
  }

  Serial.printf("[BLE] RSSI=%d dBm, heap=%u, existing bonds=%d\n",
                gClient->getRssi(), ESP.getFreeHeap(), NimBLEDevice::getNumBonds());
  const bool secured = gClient->secureConnection();
  Serial.printf("[SEC] security request %s\n", secured ? "OK" : "FAILED/NOT REQUIRED");
  if (!gClient->isConnected()) {
    Serial.println("[BLE] controller disconnected during security setup");
    discardClient();
    return false;
  }

  dumpGattDatabase(gClient);
  NimBLEDevice::getScan()->clearResults();
  gAdvertisedDevice = nullptr;
  return true;
}

void printHelp() {
  Serial.println("Commands:");
  Serial.println("  g - rediscover and dump GATT database");
  Serial.println("  r - disconnect and rescan");
  Serial.println("  e - erase ESP32 BLE bonds, disconnect and rescan");
  Serial.println("  ? - print this help");
}

void requestReconnect(bool eraseBonds) {
  if ((gClient != nullptr) && gClient->isConnected()) {
    gClient->disconnect();
  }
  discardClient();
  if (eraseBonds) {
    NimBLEDevice::deleteAllBonds();
    Serial.println("[SEC] all local BLE bonds erased");
  }
  gDisconnected = false;
  gConnectRequested = false;
  gNextScanAt = millis() + 250U;
}

void processSerialCommands() {
  while (Serial.available() > 0) {
    const char command = static_cast<char>(Serial.read());
    switch (command) {
      case 'g':
      case 'G':
        if ((gClient != nullptr) && gClient->isConnected()) {
          dumpGattDatabase(gClient);
        } else {
          Serial.println("[GATT] no connected controller");
        }
        break;
      case 'r':
      case 'R':
        requestReconnect(false);
        break;
      case 'e':
      case 'E':
        requestReconnect(true);
        break;
      case '?':
        printHelp();
        break;
      case '\r':
      case '\n':
        break;
      default:
        Serial.printf("Unknown command '%c'. Enter ? for help.\n", command);
        break;
    }
  }
}

}  // namespace

void setup() {
  Serial.begin(115200);
  delay(800);

  Serial.println();
  Serial.println("Flydigi Direwolf 3 BLE/GATT probe");
  Serial.println("Build: gamepad_probe (production web controller is excluded)");
  Serial.printf("Target FN+A: %s, FN+Y: %s\n", kDInputAddress,
                kFlashplayAddress);

  NimBLEDevice::init("STM32-KIT Gamepad Probe");
  NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  NimBLEDevice::setSecurityIOCap(BLE_HS_IO_NO_INPUT_OUTPUT);
  NimBLEDevice::setSecurityAuth(true, false, false);

  Serial.printf("Local BLE address: %s, bonds=%d, heap=%u\n",
                NimBLEDevice::getAddress().toString().c_str(),
                NimBLEDevice::getNumBonds(), ESP.getFreeHeap());
  printHelp();
  startScan();
}

void loop() {
  processSerialCommands();

  if (gConnectRequested) {
    gConnectRequested = false;
    if (!connectTarget()) {
      gNextScanAt = millis() + kRetryDelayMs;
    }
  }

  if (gDisconnected) {
    gDisconnected = false;
    discardClient();
    gNextScanAt = millis() + kRetryDelayMs;
  }

  const uint32_t now = millis();
  const bool connected = (gClient != nullptr) && gClient->isConnected();
  if (!connected && !gScanRunning && !gConnectRequested &&
      (static_cast<int32_t>(now - gNextScanAt) >= 0)) {
    startScan();
  }

  if (connected && ((now - gLastStatusAt) >= kStatusIntervalMs)) {
    gLastStatusAt = now;
    Serial.printf("[STATUS] connected=1 RSSI=%d heap=%u subscriptions=%u "
                  "notifications=%lu duplicates=%lu\n",
                  gClient->getRssi(), ESP.getFreeHeap(), gSubscriptionCount,
                  static_cast<unsigned long>(gNotificationCount),
                  static_cast<unsigned long>(gDuplicateCount));
  }

  delay(2);
}
