#include "serial_protocol.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include <ctype.h>
#include <stdlib.h>
#include <string.h>

namespace refactored {

SerialProtocol::SerialProtocol() : m_buffer(), m_len(0) {}

void SerialProtocol::trimToken(char* token) {
  if (token == NULL) return;

  size_t lead = 0;
  while (token[lead] == ' ' || token[lead] == '\t') {
    ++lead;
  }
  if (lead > 0) {
    memmove(token, token + lead, strlen(token + lead) + 1);
  }

  size_t len = strlen(token);
  while (len > 0 && (token[len - 1] == ' ' || token[len - 1] == '\t' || token[len - 1] == '\r')) {
    token[len - 1] = '\0';
    --len;
  }
}

void SerialProtocol::toUpperInPlace(char* token) {
  if (token == NULL) return;
  for (size_t i = 0; token[i] != '\0'; ++i) {
    token[i] = static_cast<char>(toupper(token[i]));
  }
}

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

bool SerialProtocol::parseFromCString(char* line, SerialCommand& outCommand) {
  outCommand.action = CommandAction::INVALID;
  outCommand.motor = MotorSelector::NONE;
  outCommand.targetTicks = 0;
  outCommand.speedTicksPerSecond = 0.0f;
  outCommand.auxTicks = 0;

  char* first = strtok(line, ",");
  if (first == NULL) return false;

  trimToken(first);

  // Legacy command support: REQUEST,<motor>,<target>
  char legacyCheck[24];
  strncpy(legacyCheck, first, sizeof(legacyCheck) - 1);
  legacyCheck[sizeof(legacyCheck) - 1] = '\0';
  toUpperInPlace(legacyCheck);
  if (strcmp(legacyCheck, "REQUEST") == 0) {
    char* motorToken = strtok(NULL, ",");
    char* targetToken = strtok(NULL, ",");
    if (motorToken == NULL || targetToken == NULL) return false;

    MotorSelector motor = parseMotorToken(motorToken);
    if (motor == MotorSelector::NONE) return false;

    char* endPtr = NULL;
    const long target = strtol(targetToken, &endPtr, 10);
    if (endPtr == targetToken) return false;

    outCommand.action = CommandAction::MOVE_ABSOLUTE;
    outCommand.motor = motor;
    outCommand.targetTicks = target;
    return true;
  }

  // Main protocol:
  // 0
  // 1,<MOTOR>,<ABS_TICKS>
  // 2,FOC,<ABS_TICKS>,<TICKS_PER_SEC>
  // 3
  // 4,<MOTOR>
  // 5,<MOTOR>[,DELTA_TICKS]
  char* endPtr = NULL;
  const long actionValue = strtol(first, &endPtr, 10);
  if (endPtr == first) return false;

  if (actionValue == static_cast<long>(CommandAction::STOP_ALL)) {
    outCommand.action = CommandAction::STOP_ALL;
    return true;
  }

  if (actionValue == static_cast<long>(CommandAction::CALIBRATE_ALL)) {
    outCommand.action = CommandAction::CALIBRATE_ALL;
    return true;
  }

  if (actionValue == static_cast<long>(CommandAction::MOTION_SELF_TEST)) {
    char* motorToken = strtok(NULL, ",");
    if (motorToken == NULL) return false;
    MotorSelector motor = parseMotorToken(motorToken);
    if (motor == MotorSelector::NONE) return false;
    outCommand.action = CommandAction::MOTION_SELF_TEST;
    outCommand.motor = motor;

    char* deltaToken = strtok(NULL, ",");
    if (deltaToken != NULL) {
      char* deltaEnd = NULL;
      const long delta = strtol(deltaToken, &deltaEnd, 10);
      if (deltaEnd == deltaToken) return false;
      outCommand.auxTicks = delta;
    }
    return true;
  }

  if (actionValue == static_cast<long>(CommandAction::MOVE_ABSOLUTE) ||
      actionValue == static_cast<long>(CommandAction::MOVE_WITH_SPEED) ||
      actionValue == static_cast<long>(CommandAction::CALIBRATE_ONE)) {
    char* motorToken = strtok(NULL, ",");
    if (motorToken == NULL) return false;

    MotorSelector motor = parseMotorToken(motorToken);
    if (motor == MotorSelector::NONE) return false;
    outCommand.motor = motor;

    if (actionValue == static_cast<long>(CommandAction::CALIBRATE_ONE)) {
      outCommand.action = CommandAction::CALIBRATE_ONE;
      return true;
    }

    char* targetToken = strtok(NULL, ",");
    if (targetToken == NULL) return false;

    char* targetEnd = NULL;
    const long target = strtol(targetToken, &targetEnd, 10);
    if (targetEnd == targetToken) return false;
    outCommand.targetTicks = target;

    if (actionValue == static_cast<long>(CommandAction::MOVE_ABSOLUTE)) {
      outCommand.action = CommandAction::MOVE_ABSOLUTE;
      return true;
    }

    char* speedToken = strtok(NULL, ",");
    if (speedToken == NULL) return false;
    char* speedEnd = NULL;
    const float speed = static_cast<float>(strtod(speedToken, &speedEnd));
    if (speedEnd == speedToken) return false;

    outCommand.action = CommandAction::MOVE_WITH_SPEED;
    outCommand.speedTicksPerSecond = speed;
    return true;
  }

  return false;
}

ParseStatus SerialProtocol::pollCommand(SerialCommand& outCommand) {
  while (Serial.available() > 0) {
    const char c = static_cast<char>(Serial.read());
    if (c == '\r') {
      continue;
    }

    if (c == '\n') {
      if (m_len == 0) {
        continue;
      }

      m_buffer[m_len] = '\0';
      m_len = 0;
      return parseFromCString(m_buffer, outCommand) ? ParseStatus::VALID_LINE : ParseStatus::INVALID_LINE;
    }

    if (m_len < (kBufferSize - 1)) {
      m_buffer[m_len++] = c;
    } else {
      // Overflow protection: drop line and reset parser state.
      m_len = 0;
      return ParseStatus::INVALID_LINE;
    }
  }
  return ParseStatus::NO_LINE;
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
