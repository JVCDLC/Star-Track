#ifndef INTERRUPT_HUB_HPP
#define INTERRUPT_HUB_HPP

#include <Arduino.h>

#include "refactored_config.hpp"

namespace refactored {

class InterruptHub {
 public:
  static void begin();

  static long readEncoderTicks(MotorId id);
  static void writeEncoderTicks(MotorId id, long ticks);

  static bool isSwitchTriggered(SwitchId id);
  static uint16_t readSwitchBitmap();

  // ISR entry helpers kept public so vector functions can dispatch here
  // without exposing internal state arrays directly.
  static void handlePortBInterrupt(uint8_t currentPortB);
  static void handlePortKInterrupt(uint8_t currentPortK);

 private:
  static void isrDecEncoder();
  static void isrRaEncoder();

  static void updateSwitchBitFromIsr(uint8_t index, bool active);
  static void decodeEncoderStepFromIsr(MotorId id, uint8_t newState);

  static volatile long s_encoderTicks[static_cast<uint8_t>(MotorId::COUNT)];
  static volatile uint8_t s_encoderPrevState[static_cast<uint8_t>(MotorId::COUNT)];
  static volatile uint16_t s_switchBitmap;
  static volatile uint8_t s_lastPortB;
  static volatile uint8_t s_lastPortK;
};

}  // namespace refactored

#endif  // INTERRUPT_HUB_HPP
