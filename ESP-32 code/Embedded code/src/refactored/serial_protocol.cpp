#include "serial_protocol.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace refactored {

/**
 * @brief Constructor for SerialProtocol.
 * Initializes the buffer to empty state.
 */
SerialProtocol::SerialProtocol() : m_buffer(), m_len(0) {}

/**
 * @brief Trim leading and trailing whitespace from a token string.
 * @param token Pointer to the null-terminated string to trim. Modified in-place.
 * 
 * This function removes spaces and tabs from both ends of the string.
 * Used to clean up tokens parsed from comma-separated input.
 */
void SerialProtocol::trimToken(char* token) {
  if (token == NULL) return;

  // Remove leading whitespace
  size_t lead = 0;
  while (token[lead] == ' ' || token[lead] == '\t') {
    ++lead;
  }
  if (lead > 0) {
    memmove(token, token + lead, strlen(token + lead) + 1);
  }

  // Remove trailing whitespace (including carriage return)
  size_t len = strlen(token);
  while (len > 0 && (token[len - 1] == ' ' || token[len - 1] == '\t' || token[len - 1] == '\r')) {
    token[len - 1] = '\0';
    --len;
  }
}

/**
 * @brief Convert all characters in a token to uppercase in-place.
 * @param token Pointer to the null-terminated string to convert. Modified in-place.
 * 
 * Used to normalize motor names for case-insensitive comparison.
 */
void SerialProtocol::toUpperInPlace(char* token) {
  if (token == NULL) return;
  for (size_t i = 0; token[i] != '\0'; ++i) {
    token[i] = static_cast<char>(toupper(token[i]));
  }
}

/**
 * @brief Parse a motor selector token into the corresponding enum value.
 * @param token The string token to parse (will be trimmed and uppercased).
 * @return The MotorSelector enum value, or NONE if unrecognized.
 * 
 * Supports multiple aliases for each motor:
 * - RA: "RA", "RIGHT_ASCENSION"
 * - DEC: "DEC", "DECLIMATION" (typo), "DECLINATION"
 * - FOC: "FOC", "FOCUS"
 */
MotorSelector SerialProtocol::parseMotorToken(char* token) {
  if (token == NULL) return MotorSelector::NONE;
  trimToken(token);
  toUpperInPlace(token);

  if (strcmp(token, "RA") == 0 || strcmp(token, "RIGHT_ASCENSION") == 0) {
    return MotorSelector::MOTOR_RA;
  }
  if (strcmp(token, "DEC") == 0 || strcmp(token, "DECLIMATION") == 0 || strcmp(token, "DECLINATION") == 0) {
    return MotorSelector::MOTOR_DEC;
  }
  if (strcmp(token, "FOC") == 0 || strcmp(token, "FOCUS") == 0) {
    return MotorSelector::MOTOR_FOC;
  }
  return MotorSelector::NONE;
}

/**
 * @brief Parse a serial command from a null-terminated C-string.
 * @param line The input string containing the command (modified during parsing).
 * @param outCommand Output structure filled with parsed command data.
 * @return true if parsing succeeded and a valid command was found, false otherwise.
 * 
 * This function implements the main command protocol parsing. It supports:
 * - Legacy format: "REQUEST,<motor>,<target>"
 * - New format: numeric action code followed by comma-separated parameters
 * 
 * The function uses strtok to split the input string, which modifies the original string.
 * All numeric parsing uses strtol/strtod for robustness.
 */
bool SerialProtocol::parseFromCString(char* line, SerialCommand& outCommand) {
  // Initialize output to invalid state
  outCommand.action = CommandAction::INVALID;
  outCommand.motor = MotorSelector::NONE;
  outCommand.targetTicks = 0;
  outCommand.speedTicksPerSecond = 0.0f;
  outCommand.auxTicks = 0;

  // Get first token (action or legacy keyword)
  char* first = strtok(line, ",");
  if (first == NULL) return false;

  trimToken(first);

  // Check for legacy command format: REQUEST,<motor>,<target>
  char legacyCheck[24];
  strncpy(legacyCheck, first, sizeof(legacyCheck) - 1);
  legacyCheck[sizeof(legacyCheck) - 1] = '\0';
  toUpperInPlace(legacyCheck);
  if (strcmp(legacyCheck, "REQUEST") == 0) {
    // Parse legacy format
    char* motorToken = strtok(NULL, ",");
    char* targetToken = strtok(NULL, ",");
    if (motorToken == NULL || targetToken == NULL) return false;

    // Parse motor selector
    MotorSelector motor = parseMotorToken(motorToken);
    if (motor == MotorSelector::NONE) return false;

    // Parse target position
    char* endPtr = NULL;
    const long target = strtol(targetToken, &endPtr, 10);
    if (endPtr == targetToken) return false;  // No digits found

    // Fill output command
    outCommand.action = CommandAction::MOVE_ABSOLUTE;
    outCommand.motor = motor;
    outCommand.targetTicks = target;
    return true;
  }

  // Parse main protocol (numeric action codes)
  // Supported formats:
  // 0                              # STOP_ALL
  // 1,<MOTOR>,<ABS_TICKS>          # MOVE_ABSOLUTE
  // 2,FOC,<ABS_TICKS>,<SPEED>      # MOVE_WITH_SPEED (focus only)
  // 3                              # CALIBRATE_ALL
  // 4,<MOTOR>                      # CALIBRATE_ONE
  // 5,<MOTOR>[,DELTA_TICKS]        # MOTION_SELF_TEST
  char* endPtr = NULL;
  const long actionValue = strtol(first, &endPtr, 10);
  if (endPtr == first) return false;  // Not a valid number

  // Handle commands without parameters
  if (actionValue == static_cast<long>(CommandAction::STOP_ALL)) {
    outCommand.action = CommandAction::STOP_ALL;
    return true;
  }

  if (actionValue == static_cast<long>(CommandAction::CALIBRATE_ALL)) {
    outCommand.action = CommandAction::CALIBRATE_ALL;
    return true;
  }

  // Handle motion self-test: 5,<MOTOR>[,DELTA_TICKS]
  if (actionValue == static_cast<long>(CommandAction::MOTION_SELF_TEST)) {
    char* motorToken = strtok(NULL, ",");
    if (motorToken == NULL) return false;
    MotorSelector motor = parseMotorToken(motorToken);
    if (motor == MotorSelector::NONE) return false;
    outCommand.action = CommandAction::MOTION_SELF_TEST;
    outCommand.motor = motor;

    // Optional delta parameter
    char* deltaToken = strtok(NULL, ",");
    if (deltaToken != NULL) {
      char* deltaEnd = NULL;
      const long delta = strtol(deltaToken, &deltaEnd, 10);
      if (deltaEnd == deltaToken) return false;
      outCommand.auxTicks = delta;
    }
    return true;
  }

  // Handle commands with motor parameter
  if (actionValue == static_cast<long>(CommandAction::MOVE_ABSOLUTE) ||
      actionValue == static_cast<long>(CommandAction::MOVE_WITH_SPEED) ||
      actionValue == static_cast<long>(CommandAction::CALIBRATE_ONE) ||
      actionValue == static_cast<long>(CommandAction::DEBUG_FOCUS)) {
    
    // Parse motor selector
    char* motorToken = strtok(NULL, ",");
    if (motorToken == NULL) return false;

    MotorSelector motor = parseMotorToken(motorToken);
    if (motor == MotorSelector::NONE) return false;
    outCommand.motor = motor;

    // Calibration one doesn't need more parameters
    if (actionValue == static_cast<long>(CommandAction::CALIBRATE_ONE)) {
      outCommand.action = CommandAction::CALIBRATE_ONE;
      return true;
    }

    // Debug focus doesn't need more parameters
    if (actionValue == static_cast<long>(CommandAction::DEBUG_FOCUS)) {
      outCommand.action = CommandAction::DEBUG_FOCUS;
      return true;
    }

    // Parse target position
    char* targetToken = strtok(NULL, ",");
    if (targetToken == NULL) return false;

    char* targetEnd = NULL;
    const long target = strtol(targetToken, &targetEnd, 10);
    if (targetEnd == targetToken) return false;
    outCommand.targetTicks = target;

    // Absolute move doesn't need speed
    if (actionValue == static_cast<long>(CommandAction::MOVE_ABSOLUTE)) {
      outCommand.action = CommandAction::MOVE_ABSOLUTE;
      return true;
    }

    // Parse speed for move with speed (focus only)
    char* speedToken = strtok(NULL, ",");
    if (speedToken == NULL) return false;
    char* speedEnd = NULL;
    const float speed = static_cast<float>(strtod(speedToken, &speedEnd));
    if (speedEnd == speedToken) return false;

    outCommand.action = CommandAction::MOVE_WITH_SPEED;
    outCommand.speedTicksPerSecond = speed;
    return true;
  }

  // Unknown action code
  return false;
}

/**
 * @brief Poll the serial port for incoming data and attempt to parse a complete command.
 * @param outCommand Output structure filled with parsed command if a complete line is received.
 * @return ParseStatus indicating the result:
 *         - NO_LINE: No complete line received yet
 *         - VALID_LINE: A complete, valid command line was parsed
 *         - INVALID_LINE: A complete line was received but parsing failed
 * 
 * This function accumulates incoming serial characters into a buffer until a newline is received.
 * It handles carriage returns by ignoring them, and protects against buffer overflow.
 * When a newline is received, it attempts to parse the accumulated line.
 * 
 * Input: Serial data from Arduino Serial port
 * Output: Parsed command in outCommand, or status indicating parsing state
 * 
 * The buffer is fixed-size (96 bytes) to prevent heap fragmentation in embedded systems.
 */
ParseStatus SerialProtocol::pollCommand(SerialCommand& outCommand) {
  // Process all available serial characters
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    
    // Ignore carriage returns (Windows line endings)
    if (c == '\r') {
      continue;
    }

    // Newline indicates end of command line
    if (c == '\n') {
      // Empty line - ignore
      if (m_len == 0) {
        continue;
      }

      // Null-terminate the buffer and reset length
      m_buffer[m_len] = '\0';
      m_len = 0;
      
      // Attempt to parse the complete line
      return parseFromCString(m_buffer, outCommand) ? ParseStatus::VALID_LINE : ParseStatus::INVALID_LINE;
    }

    // Add character to buffer if space available
    if (m_len < (kBufferSize - 1)) {
      m_buffer[m_len++] = c;
    } else {
      // Buffer overflow - discard the line and reset
      m_len = 0;
      return ParseStatus::INVALID_LINE;
    }
  }
  return ParseStatus::NO_LINE;
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
