#ifndef MOUNT_CONTROLLER_HPP
#define MOUNT_CONTROLLER_HPP

#include "bts7960_motor.hpp"
#include "md13s_motor.hpp"
#include "serial_protocol.hpp"

namespace refactored {

enum class MountState : uint8_t {
  UNINITIALIZED = 0,
  CALIBRATING,
  READY,
  TRACKING,
  ERROR
};

class MountController {
 public:
  MountController();

  void begin();
  void update();

  MountState readState() const { return m_state; }

 private:
  MotorBase* selectMotor(MotorSelector selector);
  bool processCommand(const SerialCommand& cmd);
  void updateMountState();
  void emitDebug(unsigned long nowMs);
  void updateMotionSelfTest(unsigned long nowMs);

  void startParallelCalibration();
  bool startSingleMotorCalibration(MotorSelector selector);
  void emergencyStopAll();
  bool startMotionSelfTest(MotorSelector selector, long requestedDeltaTicks);

  SerialProtocol m_serial;
  BTS7960Motor m_motorDec;
  BTS7960Motor m_motorRa;
  MD13SMotor m_motorFoc;
  MountState m_state;
  unsigned long m_lastDebugMs;
  MountState m_lastPrintedMountState;
  MotorState m_lastPrintedRaState;
  MotorState m_lastPrintedDecState;
  MotorState m_lastPrintedFocState;

  struct MotionSelfTest {
    bool active;
    MotorBase* motor;
    long startTicks;
    long minExpectedDelta;
    unsigned long startMs;
    unsigned long timeoutMs;
  } m_selfTest;
};

}  // namespace refactored

#endif  // MOUNT_CONTROLLER_HPP
