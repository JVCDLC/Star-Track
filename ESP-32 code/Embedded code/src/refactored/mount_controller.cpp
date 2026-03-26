#include "mount_controller.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include "interrupt_hub.hpp"
#include "refactored_config.hpp"

namespace refactored {
namespace {

static const bool kDebugLogsEnabled = true;
static const unsigned long kDebugPeriodMs = 6000;

PidConfig buildDecPid() {
  PidConfig cfg;
  cfg.kp = 0.21f;
  cfg.ki = 0.00021f;
  cfg.kd = 0.000008f;
  cfg.integralMin = -800.0f;
  cfg.integralMax = 800.0f;
  return cfg;
}

PidConfig buildRaPid() {
  PidConfig cfg;
  cfg.kp = 0.16f;
  cfg.ki = 0.00008f;
  cfg.kd = 0.0002f;
  cfg.integralMin = -1000.0f;
  cfg.integralMax = 1000.0f;
  return cfg;
}

PidConfig buildFocusPositionPid() {
  PidConfig cfg;
  cfg.kp = 0.10f;
  cfg.ki = 0.05f;
  cfg.kd = 0.0f;
  cfg.integralMin = -500.0f;
  cfg.integralMax = 500.0f;
  return cfg;
}

SpeedPidConfig buildFocusSpeedPid() {
  SpeedPidConfig cfg;
  cfg.kp = 2.8f;
  cfg.ki = 0.25f;
  cfg.kd = 0.04f;
  cfg.integralMin = -400.0f;
  cfg.integralMax = 400.0f;
  cfg.positionAssistKp = 0.8f;
  cfg.slowdownWindowTicks = 30;
  cfg.minSpeedTicksPerSec = 0.8f;
  return cfg;
}

MotorRuntimeConfig buildRuntimeCommon() {
  MotorRuntimeConfig cfg;
  cfg.pwmMax = 255;
  cfg.controlPeriodMs = 5;
  cfg.noMotionTimeoutMs = 2500;
  cfg.atTargetToleranceTicks = 0;
  return cfg;
}

CalibrationConfig buildCalibrationFast() {
  CalibrationConfig cfg;
  // RA/DEC: aggressive 2-pass end-stop detection
  // (full-speed hit, backoff, slow verify hit, backoff).
  cfg.fastSeekPwm = 255;
  cfg.verifySeekPwm = 95;
  cfg.backoffPwm = 100;
  cfg.backoffDurationMs = 1000;
  cfg.releaseStableMs = 40;
  cfg.verifyDeltaToleranceTicks = 8;
  cfg.singleSwitchHomeOnly = false;
  cfg.centerToleranceTicks = 2;
  cfg.centerSettleMs = 80;
  cfg.stictionScanStartPwm = 8;
  cfg.stictionScanStepPwm = 1;
  cfg.stictionScanMaxPwm = 180;
  cfg.stictionSampleWindowMs = 120;
  cfg.stictionDetectTicks = 3;
  return cfg;
}

CalibrationConfig buildCalibrationFocus() {
  CalibrationConfig cfg;
  // Focus: single-switch homing only.
  // After homing to SW1, the controller waits in READY for higher-level commands.
  cfg.fastSeekPwm = 150;
  cfg.verifySeekPwm = 65;
  cfg.backoffPwm = 65;
  cfg.backoffDurationMs = 180;
  cfg.releaseStableMs = 40;
  cfg.verifyDeltaToleranceTicks = 6;
  cfg.singleSwitchHomeOnly = true;
  cfg.centerToleranceTicks = 1;
  cfg.centerSettleMs = 80;
  cfg.stictionScanStartPwm = 5;
  cfg.stictionScanStepPwm = 1;
  cfg.stictionScanMaxPwm = 140;
  cfg.stictionSampleWindowMs = 120;
  cfg.stictionDetectTicks = 2;
  return cfg;
}

PrecisionConfig buildPrecisionCommon() {
  PrecisionConfig cfg;
  cfg.overshootWindowTicks = 1;
  cfg.reverseKickMs = 20;
  cfg.reverseKickExtraPwm = 24;
  cfg.adaptiveRaiseDelayMs = 120;
  cfg.adaptiveRaiseStep = 1;
  cfg.adaptiveDropStep = 1;
  return cfg;
}

bool isMotorMovingState(MotorState state) {
  return state == MotorState::MOVING ||
         state == MotorState::SPEED_MOVING ||
         state == MotorState::MICRO_CORRECTION;
}

const char* motorStateText(MotorState state) {
  switch (state) {
    case MotorState::UNINITIALIZED: return "UNINITIALIZED";
    case MotorState::BACKOFF_FROM_LIMIT: return "BACKOFF_FROM_LIMIT";
    case MotorState::CALIBRATING_MIN: return "CALIBRATING_MIN_FAST";
    case MotorState::BACKOFF_AFTER_MIN: return "BACKOFF_AFTER_MIN_FAST";
    case MotorState::CALIBRATING_MIN_VERIFY: return "CALIBRATING_MIN_SLOW";
    case MotorState::BACKOFF_AFTER_MIN_VERIFY: return "BACKOFF_AFTER_MIN_SLOW";
    case MotorState::CALIBRATING_MAX: return "CALIBRATING_MAX_FAST";
    case MotorState::BACKOFF_AFTER_MAX: return "BACKOFF_AFTER_MAX_FAST";
    case MotorState::CALIBRATING_MAX_VERIFY: return "CALIBRATING_MAX_SLOW";
    case MotorState::BACKOFF_AFTER_MAX_VERIFY: return "BACKOFF_AFTER_MAX_SLOW";
    case MotorState::CENTERING: return "CENTERING";
    case MotorState::STICTION_SCAN_POS: return "STICTION_POS";
    case MotorState::STICTION_SCAN_NEG: return "STICTION_NEG";
    case MotorState::CALIB_RETURN_TO_ZERO: return "CALIB_RETURN_TO_ZERO";
    case MotorState::READY: return "READY";
    case MotorState::MOVING: return "MOVING";
    case MotorState::SPEED_MOVING: return "SPEED_MOVING";
    case MotorState::MICRO_CORRECTION: return "MICRO_CORRECTION";
    case MotorState::STOPPED: return "STOPPED";
    case MotorState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

const char* mountStateText(MountState state) {
  switch (state) {
    case MountState::UNINITIALIZED: return "UNINITIALIZED";
    case MountState::CALIBRATING: return "CALIBRATING";
    case MountState::READY: return "READY";
    case MountState::TRACKING: return "TRACKING";
    case MountState::ERROR: return "ERROR";
    default: return "UNKNOWN";
  }
}

}  // namespace

MountController::MountController()
    : m_serial(),
      m_motorDec("DEC",
                 MotorId::AXIS_DEC,
                 Bts7960Pins{pins::DEC_RPWM, pins::DEC_LPWM, pins::DEC_REN, pins::DEC_LEN},
                 buildDecPid(),
                 buildRuntimeCommon(),
                 LimitConfig{
                     // DEC hardware mapping (user-verified):
                     // - SW1: one end-stop
                     // - SW4: opposite end-stop
                     //
                     // Direction signs are kept aligned with the old behavior:
                     // +1 blocks travel toward the SW4 side, -1 blocks toward the SW1 side.
                     true,  SwitchId::SW4, +1,
                     true,  SwitchId::SW1, -1,
                     42000},
                 buildCalibrationFast(),
                 buildPrecisionCommon()),
      m_motorRa("RA",
                MotorId::AXIS_RA,
                Bts7960Pins{pins::RA_RPWM, pins::RA_LPWM, pins::RA_REN, pins::RA_LEN},
                buildRaPid(),
                buildRuntimeCommon(),
                LimitConfig{
                    true,  SwitchId::SW3, -1,
                    true,  SwitchId::SW2, +1,
                    65000},
                buildCalibrationFast(),
                buildPrecisionCommon()),
      m_motorFoc("FOC",
                 MotorId::AXIS_FOC,
                 Md13sPins{pins::FOC_PWM, pins::FOC_DIR, true},
                 buildFocusPositionPid(),
                 buildFocusSpeedPid(),
                 buildRuntimeCommon(),
                 LimitConfig{
                     // Keep focus on a dedicated single home switch (no shared DEC switch).
                     true,   SwitchId::SW6, -1,
                     false,  SwitchId::SW6, +1,
                     3500},
                 buildCalibrationFocus(),
                 buildPrecisionCommon()),
      m_state(MountState::UNINITIALIZED),
      m_lastDebugMs(0),
      m_lastPrintedMountState(MountState::UNINITIALIZED),
      m_lastPrintedRaState(MotorState::UNINITIALIZED),
      m_lastPrintedDecState(MotorState::UNINITIALIZED),
      m_lastPrintedFocState(MotorState::UNINITIALIZED),
      m_selfTest{false, NULL, 0, 0, 0, 0} {}

void MountController::begin() {
  // Silent PWM setup copied from current project behavior.
  TCCR4B = (TCCR4B & 0b11111000) | 0x01;
  TCCR1B = (TCCR1B & 0b11111000) | 0x01;
  TCCR5B = (TCCR5B & 0b11111000) | 0x01;
  TCCR2B = (TCCR2B & 0b11111000) | 0x01;

  Serial.begin(115200);
  delay(150);

  InterruptHub::begin();
  m_motorDec.begin();
  m_motorRa.begin();
  m_motorFoc.begin();

  if (kDebugLogsEnabled) {
    Serial.print(F("DBG,BOOT,sw=0x"));
    Serial.println(InterruptHub::readSwitchBitmap(), HEX);
    Serial.print(F("DBG,BOOT,SW1="));
    Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW1));
    Serial.print(F(",SW2="));
    Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW2));
    Serial.print(F(",SW3="));
    Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW3));
    Serial.print(F(",SW4="));
    Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW4));
    Serial.print(F(",SW5="));
    Serial.println(InterruptHub::isSwitchTriggered(SwitchId::SW5));
  }

  startParallelCalibration();
}

MotorBase* MountController::selectMotor(MotorSelector selector) {
  if (selector == MotorSelector::MOTOR_DEC) return &m_motorDec;
  if (selector == MotorSelector::MOTOR_RA) return &m_motorRa;
  if (selector == MotorSelector::MOTOR_FOC) return &m_motorFoc;
  return NULL;
}

void MountController::startParallelCalibration() {
  m_motorDec.requestCalibration();
  m_motorRa.requestCalibration();
  m_motorFoc.requestCalibration();
  m_state = MountState::CALIBRATING;
}

bool MountController::startSingleMotorCalibration(MotorSelector selector) {
  MotorBase* motor = selectMotor(selector);
  if (motor == NULL) {
    return false;
  }
  motor->requestCalibration();
  m_state = MountState::CALIBRATING;
  return true;
}

bool MountController::startMotionSelfTest(MotorSelector selector, long requestedDeltaTicks) {
  // Hardware motion self-test:
  // command a short move and verify encoder delta rises above threshold.
  // This separates software/control issues from wiring/driver issues quickly.
  MotorBase* motor = selectMotor(selector);
  if (motor == NULL || motor->isError()) {
    return false;
  }

  if (motor->isCalibrating()) {
    motor->emergencyStopAndHold();
  }

  long delta = requestedDeltaTicks;
  if (delta == 0) {
    delta = (selector == MotorSelector::MOTOR_FOC) ? 60 : 800;
  }
  if (delta < 0) delta = -delta;

  const long start = motor->readTicks();
  bool accepted = motor->commandPositionTicks(start + delta);
  if (!accepted) {
    accepted = motor->commandPositionTicks(start - delta);
  }
  if (!accepted) {
    return false;
  }

  m_selfTest.active = true;
  m_selfTest.motor = motor;
  m_selfTest.startTicks = start;
  m_selfTest.minExpectedDelta = (selector == MotorSelector::MOTOR_FOC) ? 8 : 20;
  m_selfTest.startMs = millis();
  m_selfTest.timeoutMs = 5000;

  if (kDebugLogsEnabled) {
    Serial.print(F("DBG,TEST,START,motor="));
    Serial.print(motor->readName());
    Serial.print(F(",targetDelta="));
    Serial.println(delta);
  }
  return true;
}

void MountController::emergencyStopAll() {
  m_motorDec.emergencyStopAndHold();
  m_motorRa.emergencyStopAndHold();
  m_motorFoc.emergencyStopAndHold();
}

bool MountController::processCommand(const SerialCommand& cmd) {
  // Protocol:
  // 0
  // 1,<RA|DEC|FOC>,<target_ticks>
  // 2,FOC,<target_ticks>,<ticks_per_second>
  // 3 (calibrate all)
  // 4,<RA|DEC|FOC> (calibrate one motor)
  // 5,<RA|DEC|FOC>[,deltaTicks] (on-board motion self-test)
  switch (cmd.action) {
    case CommandAction::STOP_ALL:
      emergencyStopAll();
      return true;

    case CommandAction::MOVE_ABSOLUTE: {
      MotorBase* motor = selectMotor(cmd.motor);
      if (motor == NULL) return false;
      const bool ok = motor->commandPositionTicks(cmd.targetTicks);
      if (!ok && kDebugLogsEnabled) {
        Serial.print(F("DBG,REJECT,MOVE_ABSOLUTE,motor="));
        Serial.print(motor->readName());
        Serial.print(F(",state="));
        Serial.print(motorStateText(motor->readState()));
        Serial.print(F(",target="));
        Serial.println(cmd.targetTicks);
      }
      return ok;
    }

    case CommandAction::MOVE_WITH_SPEED:
      if (cmd.motor != MotorSelector::MOTOR_FOC) return false;
      if (!m_motorFoc.commandSpeedTicksPerSecond(cmd.targetTicks, cmd.speedTicksPerSecond)) {
        if (kDebugLogsEnabled) {
          Serial.print(F("DBG,REJECT,MOVE_WITH_SPEED,state="));
          Serial.print(motorStateText(m_motorFoc.readState()));
          Serial.print(F(",target="));
          Serial.print(cmd.targetTicks);
          Serial.print(F(",speed="));
          Serial.println(cmd.speedTicksPerSecond, 3);
        }
        return false;
      }
      return true;

    case CommandAction::CALIBRATE_ALL:
      startParallelCalibration();
      return true;

    case CommandAction::CALIBRATE_ONE:
      if (!startSingleMotorCalibration(cmd.motor)) {
        if (kDebugLogsEnabled) {
          Serial.println(F("DBG,REJECT,CALIBRATE_ONE"));
        }
        return false;
      }
      return true;

    case CommandAction::MOTION_SELF_TEST:
      return startMotionSelfTest(cmd.motor, cmd.auxTicks);

    case CommandAction::INVALID:
    default:
      return false;
  }
}

void MountController::updateMotionSelfTest(unsigned long nowMs) {
  if (!m_selfTest.active || m_selfTest.motor == NULL) return;

  const long delta = m_selfTest.motor->readTicks() - m_selfTest.startTicks;
  const long absDelta = (delta < 0) ? -delta : delta;
  if (absDelta >= m_selfTest.minExpectedDelta) {
    if (kDebugLogsEnabled) {
      Serial.print(F("DBG,TEST,PASS,motor="));
      Serial.print(m_selfTest.motor->readName());
      Serial.print(F(",delta="));
      Serial.println(absDelta);
    }
    m_selfTest.active = false;
    return;
  }

  if ((nowMs - m_selfTest.startMs) >= m_selfTest.timeoutMs) {
    if (kDebugLogsEnabled) {
      Serial.print(F("DBG,TEST,FAIL,motor="));
      Serial.print(m_selfTest.motor->readName());
      Serial.print(F(",delta="));
      Serial.println(absDelta);
    }
    m_selfTest.active = false;
  }
}

void MountController::emitDebug(unsigned long nowMs) {
  if (!kDebugLogsEnabled) return;
  if (Serial.available() > 0) return;  // Avoid interleaving while user types commands.
  const MountState mountNow = m_state;
  const MotorState raNow = m_motorRa.readState();
  const MotorState decNow = m_motorDec.readState();
  const MotorState focNow = m_motorFoc.readState();

  const bool stateChanged =
      (mountNow != m_lastPrintedMountState) ||
      (raNow != m_lastPrintedRaState) ||
      (decNow != m_lastPrintedDecState) ||
      (focNow != m_lastPrintedFocState);
  const bool periodicHeartbeat = ((nowMs - m_lastDebugMs) >= kDebugPeriodMs);
  if (!stateChanged && !periodicHeartbeat) return;

  m_lastDebugMs = nowMs;
  m_lastPrintedMountState = mountNow;
  m_lastPrintedRaState = raNow;
  m_lastPrintedDecState = decNow;
  m_lastPrintedFocState = focNow;

  Serial.print(F("DBG,mount="));
  Serial.print(mountStateText(m_state));
  Serial.print(F(",sw=0x"));
  Serial.print(InterruptHub::readSwitchBitmap(), HEX);
  Serial.print(F(",sw1="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW1));
  Serial.print(F(",sw2="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW2));
  Serial.print(F(",sw3="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW3));
  Serial.print(F(",sw4="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW4));
  Serial.print(F(",sw5="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW5));
  Serial.print(F(",sw6="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW6));
  Serial.print(F(",sw7="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW7));
  Serial.print(F(",sw8="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW8));
  Serial.print(F(",sw9="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW9));
  Serial.print(F(",sw10="));
  Serial.print(InterruptHub::isSwitchTriggered(SwitchId::SW10));

  Serial.print(F(" | RA("));
  Serial.print(motorStateText(m_motorRa.readState()));
  Serial.print(F(") pos="));
  Serial.print(m_motorRa.readTicks());
  Serial.print(F(" tgt="));
  Serial.print(m_motorRa.readTargetTicks());

  Serial.print(F(" | DEC("));
  Serial.print(motorStateText(m_motorDec.readState()));
  Serial.print(F(") pos="));
  Serial.print(m_motorDec.readTicks());
  Serial.print(F(" tgt="));
  Serial.print(m_motorDec.readTargetTicks());

  Serial.print(F(" | FOC("));
  Serial.print(motorStateText(m_motorFoc.readState()));
  Serial.print(F(") pos="));
  Serial.print(m_motorFoc.readTicks());
  Serial.print(F(" tgt="));
  Serial.println(m_motorFoc.readTargetTicks());
}

void MountController::updateMountState() {
  if (m_motorDec.isError() || m_motorRa.isError() || m_motorFoc.isError()) {
    m_state = MountState::ERROR;
    return;
  }

  if (m_motorDec.isCalibrating() || m_motorRa.isCalibrating() || m_motorFoc.isCalibrating()) {
    m_state = MountState::CALIBRATING;
    return;
  }

  if (isMotorMovingState(m_motorDec.readState()) ||
      isMotorMovingState(m_motorRa.readState()) ||
      isMotorMovingState(m_motorFoc.readState())) {
    m_state = MountState::TRACKING;
    return;
  }

  m_state = MountState::READY;
}

void MountController::update() {
  SerialCommand command;
  while (true) {
    const ParseStatus status = m_serial.pollCommand(command);
    if (status == ParseStatus::NO_LINE) {
      break;
    }
    if (status == ParseStatus::INVALID_LINE) {
      Serial.println(0);
      continue;
    }
    Serial.println(processCommand(command) ? 1 : 0);
  }

  const unsigned long nowMs = millis();
  m_motorDec.update(nowMs);
  m_motorRa.update(nowMs);
  m_motorFoc.update(nowMs);
  updateMotionSelfTest(nowMs);
  updateMountState();
  emitDebug(nowMs);
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
