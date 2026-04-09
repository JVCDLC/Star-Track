#include "interrupt_hub.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include <util/atomic.h>

namespace refactored {

volatile long InterruptHub::s_encoderTicks[static_cast<uint8_t>(MotorId::COUNT)] = {0, 0, 0};
volatile uint8_t InterruptHub::s_encoderPrevState[static_cast<uint8_t>(MotorId::COUNT)] = {0, 0, 0};
volatile uint16_t InterruptHub::s_switchBitmap = 0;
volatile uint8_t InterruptHub::s_lastPortB = 0;
volatile uint8_t InterruptHub::s_lastPortK = 0;

static inline uint8_t motorIndex(MotorId id) {
  return static_cast<uint8_t>(id);
}

static inline uint8_t switchIndex(SwitchId id) {
  return static_cast<uint8_t>(id);
}

void InterruptHub::begin() {
  // Configure all switch pins as active-low with pull-up.
  pinMode(pins::SW1, INPUT_PULLUP);
  pinMode(pins::SW2, INPUT_PULLUP);
  pinMode(pins::SW3, INPUT_PULLUP);
  pinMode(pins::SW4, INPUT_PULLUP);
  pinMode(pins::SW5, INPUT_PULLUP);
  pinMode(pins::SW6, INPUT_PULLUP);
  pinMode(pins::SW7, INPUT_PULLUP);
  pinMode(pins::SW8, INPUT_PULLUP);
  pinMode(pins::SW9, INPUT_PULLUP);
  pinMode(pins::SW10, INPUT_PULLUP);

  // Configure encoder pins.
  pinMode(pins::DEC_ENC_A, INPUT_PULLUP);
  pinMode(pins::DEC_ENC_B, INPUT_PULLUP);
  pinMode(pins::RA_ENC_A, INPUT_PULLUP);
  pinMode(pins::RA_ENC_B, INPUT_PULLUP);
  pinMode(pins::FOC_ENC_A, INPUT_PULLUP);
  pinMode(pins::FOC_ENC_B, INPUT_PULLUP);

  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    s_encoderTicks[motorIndex(MotorId::AXIS_DEC)] = 0;
    s_encoderTicks[motorIndex(MotorId::AXIS_RA)] = 0;
    s_encoderTicks[motorIndex(MotorId::AXIS_FOC)] = 0;

    s_encoderPrevState[motorIndex(MotorId::AXIS_DEC)] =
        static_cast<uint8_t>((digitalRead(pins::DEC_ENC_A) << 1) | digitalRead(pins::DEC_ENC_B));
    s_encoderPrevState[motorIndex(MotorId::AXIS_RA)] =
        static_cast<uint8_t>((digitalRead(pins::RA_ENC_A) << 1) | digitalRead(pins::RA_ENC_B));
    // Focus encoder uses state format (b << 1 | a) to match PB2/PB3 extraction.
    s_encoderPrevState[motorIndex(MotorId::AXIS_FOC)] =
        static_cast<uint8_t>((digitalRead(pins::FOC_ENC_B) << 1) | digitalRead(pins::FOC_ENC_A));
  }

  // Initialize switch bitmap from physical pin levels.
  uint16_t initialBitmap = 0;
  initialBitmap |= (!digitalRead(pins::SW1) ? (1u << switchIndex(SwitchId::SW1)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW2) ? (1u << switchIndex(SwitchId::SW2)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW3) ? (1u << switchIndex(SwitchId::SW3)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW4) ? (1u << switchIndex(SwitchId::SW4)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW5) ? (1u << switchIndex(SwitchId::SW5)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW6) ? (1u << switchIndex(SwitchId::SW6)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW7) ? (1u << switchIndex(SwitchId::SW7)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW8) ? (1u << switchIndex(SwitchId::SW8)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW9) ? (1u << switchIndex(SwitchId::SW9)) : 0u);
  initialBitmap |= (!digitalRead(pins::SW10) ? (1u << switchIndex(SwitchId::SW10)) : 0u);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s_switchBitmap = initialBitmap; }

  // Save baseline port snapshots for PCINT edge processing.
  s_lastPortB = PINB;
  s_lastPortK = PINK;

  // External interrupts for DEC + RA quadrature channels.
  attachInterrupt(digitalPinToInterrupt(pins::DEC_ENC_A), InterruptHub::isrDecEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::DEC_ENC_B), InterruptHub::isrDecEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::RA_ENC_A), InterruptHub::isrRaEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(pins::RA_ENC_B), InterruptHub::isrRaEncoder, CHANGE);

  // Pin change interrupts:
  // - Port B: PB2/PB3 focus encoder, PB6/PB7 switches 9/10
  // - Port K: A8..A15 switches 1..8
  PCICR |= (1 << PCIE0) | (1 << PCIE2);
  PCMSK0 |= (1 << PCINT2) | (1 << PCINT3) | (1 << PCINT6) | (1 << PCINT7);
  PCMSK2 |= 0xFF;
}

long InterruptHub::readEncoderTicks(MotorId id) {
  long ticks = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { ticks = s_encoderTicks[motorIndex(id)]; }
  return ticks;
}

void InterruptHub::writeEncoderTicks(MotorId id, long ticks) {
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { s_encoderTicks[motorIndex(id)] = ticks; }
}

bool InterruptHub::isSwitchTriggered(SwitchId id) {
  uint16_t bitmap = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { bitmap = s_switchBitmap; }
  return (bitmap & (1u << switchIndex(id))) != 0;
}

uint16_t InterruptHub::readSwitchBitmap() {
  uint16_t bitmap = 0;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) { bitmap = s_switchBitmap; }
  return bitmap;
}

void InterruptHub::updateSwitchBitFromIsr(uint8_t index, bool active) {
  if (active) {
    s_switchBitmap |= (1u << index);
  } else {
    s_switchBitmap &= static_cast<uint16_t>(~(1u << index));
  }
}

void InterruptHub::decodeEncoderStepFromIsr(MotorId id, uint8_t newState) {
  const uint8_t idx = motorIndex(id);
  const uint8_t tableIndex = static_cast<uint8_t>((s_encoderPrevState[idx] << 2) | newState);
  s_encoderTicks[idx] += QUAD_TABLE[tableIndex];
  s_encoderPrevState[idx] = newState;
}

void InterruptHub::isrDecEncoder() {
  const uint8_t a = static_cast<uint8_t>(digitalRead(pins::DEC_ENC_A));
  const uint8_t b = static_cast<uint8_t>(digitalRead(pins::DEC_ENC_B));
  const uint8_t state = static_cast<uint8_t>((a << 1) | b);
  decodeEncoderStepFromIsr(MotorId::AXIS_DEC, state);
}

void InterruptHub::isrRaEncoder() {
  const uint8_t a = static_cast<uint8_t>(digitalRead(pins::RA_ENC_A));
  const uint8_t b = static_cast<uint8_t>(digitalRead(pins::RA_ENC_B));
  const uint8_t state = static_cast<uint8_t>((a << 1) | b);
  decodeEncoderStepFromIsr(MotorId::AXIS_RA, state);
}

void InterruptHub::handlePortBInterrupt(uint8_t current) {
  const uint8_t changed = static_cast<uint8_t>(current ^ s_lastPortB);
  s_lastPortB = current;

  // Switch 9/10 on PB6/PB7 (active-low).
  if (changed & (1 << 6)) {
    updateSwitchBitFromIsr(static_cast<uint8_t>(SwitchId::SW9), (current & (1 << 6)) == 0);
  }
  if (changed & (1 << 7)) {
    updateSwitchBitFromIsr(static_cast<uint8_t>(SwitchId::SW10), (current & (1 << 7)) == 0);
  }

  // Focus encoder PB2/PB3.
  if (changed & ((1 << 2) | (1 << 3))) {
    const uint8_t a = static_cast<uint8_t>((current & (1 << 2)) >> 2);
    const uint8_t b = static_cast<uint8_t>((current & (1 << 3)) >> 3);
    const uint8_t state = static_cast<uint8_t>((b << 1) | a);
    decodeEncoderStepFromIsr(MotorId::AXIS_FOC, state);
  }
}

void InterruptHub::handlePortKInterrupt(uint8_t current) {
  const uint8_t changed = static_cast<uint8_t>(current ^ s_lastPortK);
  s_lastPortK = current;

  // Switches 1..8 on A8..A15 (PK0..PK7), active-low.
  for (uint8_t bit = 0; bit < 8; ++bit) {
    if (changed & (1 << bit)) {
      updateSwitchBitFromIsr(bit, (current & (1 << bit)) == 0);
    }
  }
}

}  // namespace refactored

ISR(PCINT0_vect) {
  using namespace refactored;
  InterruptHub::handlePortBInterrupt(PINB);
}

ISR(PCINT2_vect) {
  using namespace refactored;
  InterruptHub::handlePortKInterrupt(PINK);
}

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
