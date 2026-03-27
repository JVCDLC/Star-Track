#ifndef SERIAL_PROTOCOL_HPP
#define SERIAL_PROTOCOL_HPP

#include <Arduino.h>

namespace refactored {

/**
 * @brief Enumeration of possible command actions that can be sent via serial.
 * These represent the different operations the mount controller can perform.
 */
enum class CommandAction : uint8_t {
  STOP_ALL = 0,          ///< Stop all motors immediately
  MOVE_ABSOLUTE = 1,     ///< Move a motor to an absolute position in ticks
  MOVE_WITH_SPEED = 2,   ///< Move the focus motor to a position with specified speed (focus only)
  CALIBRATE_ALL = 3,     ///< Start calibration for all motors
  CALIBRATE_ONE = 4,     ///< Start calibration for a specific motor
  MOTION_SELF_TEST = 5,  ///< Run a self-test motion for a motor
  DEBUG_FOCUS = 6,       ///< Debug focus motor: apply PWM and monitor encoder
  INVALID = 255          ///< Invalid or unrecognized command
};

/**
 * @brief Enumeration of motor selectors for commands.
 * Specifies which motor axis the command applies to.
 */
enum class MotorSelector : uint8_t {
  NONE = 0,      ///< No motor selected (invalid)
  MOTOR_RA,      ///< Right Ascension motor
  MOTOR_DEC,     ///< Declination motor
  MOTOR_FOC      ///< Focus motor
};

/**
 * @brief Structure representing a parsed serial command.
 * Contains all the information extracted from a serial command string.
 */
struct SerialCommand {
  CommandAction action;           ///< The type of action to perform
  MotorSelector motor;            ///< Which motor to apply the action to
  long targetTicks;               ///< Target position in encoder ticks (for move commands)
  float speedTicksPerSecond;      ///< Speed in ticks per second (for speed moves, focus only)
  long auxTicks;                  ///< Auxiliary ticks value (used for self-test delta)
};

/**
 * @brief Status of parsing a serial command line.
 * Indicates whether a complete line was parsed and if it was valid.
 */
enum class ParseStatus : uint8_t {
  NO_LINE = 0,      ///< No complete line received yet
  VALID_LINE,       ///< A valid command line was parsed
  INVALID_LINE      ///< A line was received but could not be parsed
};

/**
 * @brief Class for parsing serial commands from ASCII strings.
 * Handles the protocol for communicating with the mount controller via serial.
 * Uses a fixed-size buffer to avoid heap fragmentation in embedded systems.
 */
class SerialProtocol {
 public:
  SerialProtocol();

  /**
   * @brief Poll for incoming serial data and attempt to parse a command.
   * @param outCommand Output parameter filled with the parsed command if successful.
   * @return ParseStatus indicating the result of the parsing attempt.
   */
  ParseStatus pollCommand(SerialCommand& outCommand);

  /**
   * @brief Parse a command from a null-terminated C-string.
   * @param line The input string to parse (modified in-place during parsing).
   * @param outCommand Output parameter filled with the parsed command.
   * @return true if parsing succeeded, false otherwise.
   */
  bool parseFromCString(char* line, SerialCommand& outCommand);

 private:
  /**
   * @brief Trim whitespace from a token string.
   * @param token The string to trim (modified in-place).
   */
  static void trimToken(char* token);

  /**
   * @brief Convert a token to uppercase in-place.
   * @param token The string to convert (modified in-place).
   */
  static void toUpperInPlace(char* token);

  /**
   * @brief Parse a motor selector token.
   * @param token The string token to parse.
   * @return The corresponding MotorSelector enum value.
   */
  static MotorSelector parseMotorToken(char* token);

  static const uint8_t kBufferSize = 96;  ///< Size of the internal buffer for incoming data
  char m_buffer[kBufferSize];             ///< Buffer for accumulating incoming serial data
  uint8_t m_len;                          ///< Current length of data in the buffer
};

}  // namespace refactored

#endif  // SERIAL_PROTOCOL_HPP
