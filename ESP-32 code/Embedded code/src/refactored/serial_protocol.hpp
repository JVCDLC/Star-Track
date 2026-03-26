#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <Arduino.h>

namespace refactored {

enum class CommandAction : uint8_t {
  STOP_ALL = 0,
  MOVE_ABSOLUTE = 1,
  MOVE_WITH_SPEED = 2,
  CALIBRATE_ALL = 3,
  CALIBRATE_ONE = 4,
  MOTION_SELF_TEST = 5,
  INVALID = 255
};

enum class MotorSelector : uint8_t {
  NONE = 0,
  MOTOR_RA,
  MOTOR_DEC,
  MOTOR_FOC
};

struct SerialCommand {
  CommandAction action;
  MotorSelector motor;
  long targetTicks;
  float speedTicksPerSecond;
  long auxTicks;
};

enum class ParseStatus : uint8_t {
  NO_LINE = 0,
  VALID_LINE,
  INVALID_LINE
};

class SerialProtocol {
 public:
  SerialProtocol();

  // C-string parser by design (no Arduino String usage) to avoid heap fragmentation.
  // Distinguishes "no line yet" from "line received but invalid".
  ParseStatus pollCommand(SerialCommand& outCommand);
  bool parseFromCString(char* line, SerialCommand& outCommand);

 private:
  static void trimToken(char* token);
  static void toUpperInPlace(char* token);
  static MotorSelector parseMotorToken(char* token);

  static const uint8_t kBufferSize = 96;
  char m_buffer[kBufferSize];
  uint8_t m_len;
};

}  // namespace refactored

#endif  // SERIAL_PROTOCOL_HPP
