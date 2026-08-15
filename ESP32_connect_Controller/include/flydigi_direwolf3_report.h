#pragma once

#include <stddef.h>
#include <stdint.h>

namespace flydigi_direwolf3 {

constexpr size_t kInputReportLength = 20U;

enum ActionButton : uint8_t {
  kButtonA = 0x10U,
  kButtonB = 0x20U,
  kButtonX = 0x40U,
  kButtonY = 0x80U,
};

enum MiscButton : uint8_t {
  kButtonLB = 0x01U,
  kButtonRB = 0x02U,
  kButtonLT = 0x04U,
  kButtonRT = 0x08U,
  kButtonSelect = 0x10U,
  kButtonStart = 0x20U,
  kButtonLeftStick = 0x40U,
  kButtonRightStick = 0x80U,
};

enum HatDirection : uint8_t {
  kHatCentered = 0U,
  kHatUp = 1U,
  kHatUpRight = 2U,
  kHatRight = 3U,
  kHatDownRight = 4U,
  kHatDown = 5U,
  kHatDownLeft = 6U,
  kHatLeft = 7U,
  kHatUpLeft = 8U,
};

struct State {
  int8_t leftX = 0;
  int8_t leftY = 0;
  int8_t rightX = 0;
  int8_t rightY = 0;
  uint8_t hat = kHatCentered;
  uint8_t actionButtons = 0;
  uint8_t miscButtons = 0;
  uint8_t leftTrigger = 0;
  uint8_t rightTrigger = 0;
  int16_t gyroX = 0;
  int16_t gyroY = 0;
  int16_t gyroZ = 0;

  bool actionPressed(ActionButton button) const {
    return (actionButtons & static_cast<uint8_t>(button)) != 0U;
  }

  bool miscPressed(MiscButton button) const {
    return (miscButtons & static_cast<uint8_t>(button)) != 0U;
  }
};

inline int16_t readSigned16LE(const uint8_t* data) {
  return static_cast<int16_t>(static_cast<uint16_t>(data[0]) |
                              (static_cast<uint16_t>(data[1]) << 8U));
}

inline bool parseInputReport(const uint8_t* data, size_t length, State* state) {
  if ((data == nullptr) || (state == nullptr) ||
      (length != kInputReportLength)) {
    return false;
  }

  state->leftX = static_cast<int8_t>(data[0]);
  state->leftY = static_cast<int8_t>(data[1]);
  state->rightX = static_cast<int8_t>(data[2]);
  state->rightY = static_cast<int8_t>(data[3]);
  state->hat = data[8] & 0x0FU;
  state->actionButtons = data[8] & 0xF0U;
  state->miscButtons = data[9];
  state->leftTrigger = data[12];
  state->rightTrigger = data[13];
  state->gyroX = readSigned16LE(&data[14]);
  state->gyroY = readSigned16LE(&data[16]);
  state->gyroZ = readSigned16LE(&data[18]);
  return true;
}

inline int8_t axisPercent(int8_t raw, uint8_t deadzone = 4U) {
  const int16_t value = static_cast<int16_t>(raw);
  const int16_t magnitude = (value < 0) ? -value : value;
  if (magnitude <= static_cast<int16_t>(deadzone)) {
    return 0;
  }

  const int16_t divisor = (value < 0) ? 128 : 127;
  int16_t scaled = static_cast<int16_t>((value * 100) / divisor);
  if (scaled > 100) {
    scaled = 100;
  } else if (scaled < -100) {
    scaled = -100;
  }
  return static_cast<int8_t>(scaled);
}

inline const char* hatName(uint8_t hat) {
  switch (hat) {
    case kHatCentered:
      return "CENTER";
    case kHatUp:
      return "UP";
    case kHatUpRight:
      return "UP-RIGHT";
    case kHatRight:
      return "RIGHT";
    case kHatDownRight:
      return "DOWN-RIGHT";
    case kHatDown:
      return "DOWN";
    case kHatDownLeft:
      return "DOWN-LEFT";
    case kHatLeft:
      return "LEFT";
    case kHatUpLeft:
      return "UP-LEFT";
    default:
      return "INVALID";
  }
}

}  // namespace flydigi_direwolf3
