#include "motor_base.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include <math.h>

namespace refactored {

MotorBase::MotorBase(const char* name,
                     MotorId encoderId,
                     const PidConfig& pid,
                     const MotorRuntimeConfig& runtime,
                     const LimitConfig& limits,
                     const CalibrationConfig& calibration,
                     const PrecisionConfig& precision)
    : m_name(name),
      m_encoderId(encoderId),
      m_pid(pid),
      m_runtime(runtime),
      m_limits(limits),
      m_calibration(calibration),
      m_precision(precision),
      m_motionMode(MotionMode::POSITION),
      m_state(MotorState::UNINITIALIZED),
      m_limitsKnown(false),
      m_limitMinTick(0),
      m_limitMaxTick(0),
      m_targetTicks(0),
      m_errorIntegral(0.0f),
      m_prevErrorTicks(0),
      m_lastControlMs(0),
      m_lastMotionMs(0),
      m_lastObservedTick(0),
      m_lastAppliedDirection(0),
      m_stictionMinPwmPos(calibration.stictionScanStartPwm),
      m_stictionMinPwmNeg(calibration.stictionScanStartPwm),
      m_adaptiveMinPwmPos(calibration.stictionScanStartPwm),
      m_adaptiveMinPwmNeg(calibration.stictionScanStartPwm),
      m_calCapturedMinTick(0),
      m_calCapturedMaxTick(0),
      m_calFirstHitTick(0),
      m_calCenterTick(0),
      m_calStateStartMs(0),
      m_releaseStableStartMs(0),
      m_centerStableStartMs(0),
      m_scanStepActive(false),
      m_scanPwm(0),
      m_scanStartTick(0),
      m_scanStepStartMs(0),
      m_microStartMs(0),
      m_microDirection(0) {}

int8_t MotorBase::signOf(long value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

long MotorBase::absLong(long value) {
  return (value < 0) ? -value : value;
}

void MotorBase::begin() {
  setupDriverPins();
  applyDriverPwm(0);
  m_lastAppliedDirection = 0;
  m_targetTicks = readTicks();
  m_lastObservedTick = m_targetTicks;
  m_lastControlMs = millis();
  m_lastMotionMs = m_lastControlMs;
  m_state = MotorState::STOPPED;
}

void MotorBase::requestCalibration() {
  resetController();
  m_limitsKnown = false;
  m_state = MotorState::BACKOFF_FROM_LIMIT;
  m_motionMode = MotionMode::POSITION;
  m_calStateStartMs = millis();
  m_releaseStableStartMs = 0;
  m_centerStableStartMs = 0;
  m_scanStepActive = false;
  m_scanPwm = 0;
  m_calFirstHitTick = readTicks();
  m_microDirection = 0;
  applyDriverPwm(0);
  m_lastAppliedDirection = 0;
}

bool MotorBase::commandPositionTicks(long targetTicks) {
  if (isError()) {
    return false;
  }
  if (!targetInsideKnownLimits(targetTicks)) {
    return false;
  }

  // Allow command-driven interruption of calibration to support per-axis testing.
  if (isCalibrating()) {
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;
    m_scanStepActive = false;
  }

  m_targetTicks = targetTicks;
  m_motionMode = MotionMode::POSITION;
  resetController();

  if (readTicks() == m_targetTicks) {
    m_state = MotorState::READY;
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;
  } else {
    m_state = MotorState::MOVING;
  }
  return true;
}

bool MotorBase::commandSpeedTicksPerSecond(long, float) {
  return false;
}

void MotorBase::emergencyStopAndHold() {
  const long nowPos = readTicks();
  m_targetTicks = nowPos;
  m_motionMode = MotionMode::POSITION;
  m_state = MotorState::READY;
  resetController();
  applyDriverPwm(0);
  m_lastAppliedDirection = 0;
}

long MotorBase::readTicks() const {
  return InterruptHub::readEncoderTicks(m_encoderId);
}

void MotorBase::writeTicks(long ticks) {
  InterruptHub::writeEncoderTicks(m_encoderId, ticks);
}

bool MotorBase::isCalibrating() const {
  return m_state == MotorState::BACKOFF_FROM_LIMIT ||
         m_state == MotorState::CALIBRATING_MIN ||
         m_state == MotorState::BACKOFF_AFTER_MIN ||
         m_state == MotorState::CALIBRATING_MIN_VERIFY ||
         m_state == MotorState::BACKOFF_AFTER_MIN_VERIFY ||
         m_state == MotorState::CALIBRATING_MAX ||
         m_state == MotorState::BACKOFF_AFTER_MAX ||
         m_state == MotorState::CALIBRATING_MAX_VERIFY ||
         m_state == MotorState::BACKOFF_AFTER_MAX_VERIFY ||
         m_state == MotorState::CENTERING ||
         m_state == MotorState::STICTION_SCAN_POS ||
         m_state == MotorState::STICTION_SCAN_NEG ||
         m_state == MotorState::CALIB_RETURN_TO_ZERO;
}

void MotorBase::resetController() {
  m_errorIntegral = 0.0f;
  m_prevErrorTicks = 0;
  m_lastAppliedDirection = 0;
  m_microDirection = 0;
}

void MotorBase::forceReadyAtCurrentPosition() {
  m_targetTicks = readTicks();
  m_motionMode = MotionMode::POSITION;
  m_state = MotorState::READY;
  resetController();
  applyDriverPwm(0);
  m_lastAppliedDirection = 0;
}

uint8_t MotorBase::readAdaptiveMinPwmForDir(int8_t direction) const {
  if (direction >= 0) {
    return m_adaptiveMinPwmPos;
  }
  return m_adaptiveMinPwmNeg;
}

bool MotorBase::directionBlockedByLimit(int8_t direction) const {
  if (direction == 0) {
    return false;
  }
  if (m_limits.hasMinSwitch &&
      direction == m_limits.dirTowardMin &&
      InterruptHub::isSwitchTriggered(m_limits.minSwitch)) {
    return true;
  }
  if (m_limits.hasMaxSwitch &&
      direction == m_limits.dirTowardMax &&
      InterruptHub::isSwitchTriggered(m_limits.maxSwitch)) {
    return true;
  }
  return false;
}

bool MotorBase::anyConfiguredLimitTriggered() const {
  const bool minHit = m_limits.hasMinSwitch && InterruptHub::isSwitchTriggered(m_limits.minSwitch);
  const bool maxHit = m_limits.hasMaxSwitch && InterruptHub::isSwitchTriggered(m_limits.maxSwitch);
  return minHit || maxHit;
}

bool MotorBase::targetInsideKnownLimits(long targetTicks) const {
  if (!m_limitsKnown) {
    return true;
  }
  return targetTicks >= m_limitMinTick && targetTicks <= m_limitMaxTick;
}

void MotorBase::enterError() {
  m_state = MotorState::ERROR;
  m_targetTicks = readTicks();
  m_motionMode = MotionMode::POSITION;
  resetController();
  applyDriverPwm(0);
  m_lastAppliedDirection = 0;
}

void MotorBase::driveOpenLoopDirectional(int8_t direction, uint8_t pwmMagnitude) {
  if (direction == 0 || pwmMagnitude == 0) {
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;
    return;
  }
  if (directionBlockedByLimit(direction)) {
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;
    return;
  }
  applyDriverPwm(static_cast<int16_t>(direction * pwmMagnitude));
  m_lastAppliedDirection = direction;
}

void MotorBase::update(unsigned long nowMs) {
  // Fast safety layer: enforce limit stop on every loop call, not only at PID period cadence.
  if (m_lastAppliedDirection != 0 && directionBlockedByLimit(m_lastAppliedDirection)) {
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;
    if (!isCalibrating()) {
      m_targetTicks = readTicks();
      m_motionMode = MotionMode::POSITION;
      m_state = MotorState::READY;
      resetController();
    }
  }

  const unsigned long elapsedMs = nowMs - m_lastControlMs;  // rollover-safe subtraction
  if (elapsedMs < m_runtime.controlPeriodMs) {
    return;
  }
  m_lastControlMs = nowMs;
  const float dtSeconds = (elapsedMs > 0) ? (elapsedMs * 0.001f) : 0.001f;

  if (isCalibrating()) {
    updateCalibration(nowMs);
    return;
  }
  if (m_state == MotorState::UNINITIALIZED || m_state == MotorState::ERROR || m_state == MotorState::STOPPED) {
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;
    return;
  }

  updateMotion(nowMs, dtSeconds);
}

void MotorBase::updateCalibration(unsigned long nowMs) {
  // Non-blocking calibration state machine:
  // 1) back off if boot starts on an already-triggered end-stop
  // 2) seek min and max limits
  // 3) center and re-zero encoder
  // 4) discover directional stiction thresholds
  switch (m_state) {
    case MotorState::BACKOFF_FROM_LIMIT: {
      if (anyConfiguredLimitTriggered()) {
        int8_t backoffDir = 0;
        if (m_limits.hasMinSwitch && InterruptHub::isSwitchTriggered(m_limits.minSwitch)) {
          backoffDir = static_cast<int8_t>(-m_limits.dirTowardMin);
        } else if (m_limits.hasMaxSwitch && InterruptHub::isSwitchTriggered(m_limits.maxSwitch)) {
          backoffDir = static_cast<int8_t>(-m_limits.dirTowardMax);
        }

        if (backoffDir == 0) {
          enterError();
          return;
        }

        driveOpenLoopDirectional(backoffDir, m_calibration.backoffPwm);

        // If we keep commanding backoff but encoder does not move, fail fast.
        const long movedTicks = absLong(readTicks() - m_calFirstHitTick);
        if ((nowMs - m_calStateStartMs) > (m_calibration.backoffDurationMs + 1500UL) && movedTicks < 3) {
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.println(F(",CALIB_BACKOFF_STUCK"));
          enterError();
          return;
        }
        if ((nowMs - m_calStateStartMs) > 8000UL) {
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.println(F(",CALIB_LIMIT_NOT_RELEASED"));
          enterError();
          return;
        }

        if (!anyConfiguredLimitTriggered()) {
          if (m_releaseStableStartMs == 0) {
            m_releaseStableStartMs = nowMs;
          } else if ((nowMs - m_releaseStableStartMs) >= m_calibration.releaseStableMs) {
            m_releaseStableStartMs = 0;
            applyDriverPwm(0);
  m_lastAppliedDirection = 0;
            m_state = MotorState::CALIBRATING_MIN;
          }
        } else {
          m_releaseStableStartMs = 0;
        }
      } else {
        applyDriverPwm(0);
  m_lastAppliedDirection = 0;
        m_state = MotorState::CALIBRATING_MIN;
      }
      break;
    }

    case MotorState::CALIBRATING_MIN: {
      if (!m_limits.hasMinSwitch) {
        m_calCapturedMinTick = readTicks();
        m_state = MotorState::CALIBRATING_MAX;
        break;
      }

      if (InterruptHub::isSwitchTriggered(m_limits.minSwitch)) {
        if (m_releaseStableStartMs == 0) {
          m_releaseStableStartMs = nowMs;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        } else if ((nowMs - m_releaseStableStartMs) >= m_calibration.releaseStableMs) {
          m_releaseStableStartMs = 0;
          m_calFirstHitTick = readTicks();
          m_calStateStartMs = nowMs;
          m_state = MotorState::BACKOFF_AFTER_MIN;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        }
      } else {
        m_releaseStableStartMs = 0;
        driveOpenLoopDirectional(m_limits.dirTowardMin, m_calibration.fastSeekPwm);
      }
      break;
    }

    case MotorState::BACKOFF_AFTER_MIN: {
      const int8_t backoffDir = static_cast<int8_t>(-m_limits.dirTowardMin);
      driveOpenLoopDirectional(backoffDir, m_calibration.backoffPwm);
      const bool released = !m_limits.hasMinSwitch || !InterruptHub::isSwitchTriggered(m_limits.minSwitch);
      if ((nowMs - m_calStateStartMs) >= m_calibration.backoffDurationMs && released) {
        applyDriverPwm(0);
  m_lastAppliedDirection = 0;
        m_state = MotorState::CALIBRATING_MIN_VERIFY;
      }
      break;
    }

    case MotorState::CALIBRATING_MIN_VERIFY: {
      // Second (slow) pass to capture a repeatable edge location.
      if (!m_limits.hasMinSwitch) {
        m_calCapturedMinTick = readTicks();
        m_state = MotorState::CALIBRATING_MAX;
        break;
      }

      if (InterruptHub::isSwitchTriggered(m_limits.minSwitch)) {
        if (m_releaseStableStartMs == 0) {
          m_releaseStableStartMs = nowMs;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        } else if ((nowMs - m_releaseStableStartMs) >= m_calibration.releaseStableMs) {
          m_releaseStableStartMs = 0;
          const long verifyTick = readTicks();
          const long delta = absLong(verifyTick - m_calFirstHitTick);
          m_calCapturedMinTick = verifyTick;
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.print(F(",MIN_VERIFY,first="));
          Serial.print(m_calFirstHitTick);
          Serial.print(F(",second="));
          Serial.print(verifyTick);
          Serial.print(F(",delta="));
          Serial.println(delta);
          if (delta > m_calibration.verifyDeltaToleranceTicks) {
            Serial.print(F("DBG,"));
            Serial.print(m_name);
            Serial.println(F(",MIN_VERIFY_MISMATCH"));
          }
          m_calStateStartMs = nowMs;
          m_state = MotorState::BACKOFF_AFTER_MIN_VERIFY;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        }
      } else {
        m_releaseStableStartMs = 0;
        driveOpenLoopDirectional(m_limits.dirTowardMin, m_calibration.verifySeekPwm);
      }
      break;
    }

    case MotorState::BACKOFF_AFTER_MIN_VERIFY: {
      const int8_t backoffDir = static_cast<int8_t>(-m_limits.dirTowardMin);
      driveOpenLoopDirectional(backoffDir, m_calibration.backoffPwm);
      const bool released = !m_limits.hasMinSwitch || !InterruptHub::isSwitchTriggered(m_limits.minSwitch);
      if ((nowMs - m_calStateStartMs) >= m_calibration.backoffDurationMs && released) {
        applyDriverPwm(0);
  m_lastAppliedDirection = 0;
        m_state = MotorState::CALIBRATING_MAX;
      }
      break;
    }

    case MotorState::CALIBRATING_MAX: {
      if (!m_limits.hasMaxSwitch) {
        // Single-switch mode (focus): return to home switch and stop there.
        if (m_calibration.singleSwitchHomeOnly && m_limits.hasMinSwitch) {
          if (InterruptHub::isSwitchTriggered(m_limits.minSwitch)) {
            writeTicks(0);
            m_limitsKnown = false;  // No bounded max range in one-switch mode.
            m_targetTicks = 0;
            forceReadyAtCurrentPosition();
            Serial.print(F("DBG,"));
            Serial.print(m_name);
            Serial.println(F(",HOME_DONE"));
          } else {
            driveOpenLoopDirectional(m_limits.dirTowardMin, m_calibration.verifySeekPwm);
          }
          break;
        }

        m_calCapturedMaxTick = m_calCapturedMinTick + m_limits.fallbackSpanTicks;
        const long minTick = (m_calCapturedMinTick < m_calCapturedMaxTick) ? m_calCapturedMinTick : m_calCapturedMaxTick;
        const long maxTick = (m_calCapturedMinTick > m_calCapturedMaxTick) ? m_calCapturedMinTick : m_calCapturedMaxTick;
        m_limitMinTick = minTick;
        m_limitMaxTick = maxTick;
        m_calCenterTick = minTick + ((maxTick - minTick) / 2);
        // Centering target is handled as a regular closed-loop tick target.
        // This mirrors runtime command behavior (e.g. "1,RA,target") so
        // centering speed/accuracy are consistent with normal moves.
        m_targetTicks = m_calCenterTick;
        resetController();
        m_lastObservedTick = readTicks();
        m_lastMotionMs = nowMs;
        m_state = MotorState::CENTERING;
        m_centerStableStartMs = 0;
        break;
      }

      if (InterruptHub::isSwitchTriggered(m_limits.maxSwitch)) {
        if (m_releaseStableStartMs == 0) {
          m_releaseStableStartMs = nowMs;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        } else if ((nowMs - m_releaseStableStartMs) >= m_calibration.releaseStableMs) {
          m_releaseStableStartMs = 0;
          m_calFirstHitTick = readTicks();
          m_calStateStartMs = nowMs;
          m_state = MotorState::BACKOFF_AFTER_MAX;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        }
      } else {
        m_releaseStableStartMs = 0;
        driveOpenLoopDirectional(m_limits.dirTowardMax, m_calibration.fastSeekPwm);
      }
      break;
    }

    case MotorState::BACKOFF_AFTER_MAX: {
      const int8_t backoffDir = static_cast<int8_t>(-m_limits.dirTowardMax);
      driveOpenLoopDirectional(backoffDir, m_calibration.backoffPwm);
      const bool released = !m_limits.hasMaxSwitch || !InterruptHub::isSwitchTriggered(m_limits.maxSwitch);
      if ((nowMs - m_calStateStartMs) >= m_calibration.backoffDurationMs && released) {
        applyDriverPwm(0);
  m_lastAppliedDirection = 0;
        m_state = MotorState::CALIBRATING_MAX_VERIFY;
      }
      break;
    }

    case MotorState::CALIBRATING_MAX_VERIFY: {
      // Same verification strategy for the opposite end-stop.
      if (!m_limits.hasMaxSwitch) {
        m_calCapturedMaxTick = m_calCapturedMinTick + m_limits.fallbackSpanTicks;
        m_calStateStartMs = nowMs;
        m_state = MotorState::BACKOFF_AFTER_MAX_VERIFY;
        break;
      }

      if (InterruptHub::isSwitchTriggered(m_limits.maxSwitch)) {
        if (m_releaseStableStartMs == 0) {
          m_releaseStableStartMs = nowMs;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        } else if ((nowMs - m_releaseStableStartMs) >= m_calibration.releaseStableMs) {
          m_releaseStableStartMs = 0;
          const long verifyTick = readTicks();
          const long delta = absLong(verifyTick - m_calFirstHitTick);
          m_calCapturedMaxTick = verifyTick;
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.print(F(",MAX_VERIFY,first="));
          Serial.print(m_calFirstHitTick);
          Serial.print(F(",second="));
          Serial.print(verifyTick);
          Serial.print(F(",delta="));
          Serial.println(delta);
          if (delta > m_calibration.verifyDeltaToleranceTicks) {
            Serial.print(F("DBG,"));
            Serial.print(m_name);
            Serial.println(F(",MAX_VERIFY_MISMATCH"));
          }
          m_calStateStartMs = nowMs;
          m_state = MotorState::BACKOFF_AFTER_MAX_VERIFY;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
        }
      } else {
        m_releaseStableStartMs = 0;
        driveOpenLoopDirectional(m_limits.dirTowardMax, m_calibration.verifySeekPwm);
      }
      break;
    }

    case MotorState::BACKOFF_AFTER_MAX_VERIFY: {
      const int8_t backoffDir = static_cast<int8_t>(-m_limits.dirTowardMax);
      driveOpenLoopDirectional(backoffDir, m_calibration.backoffPwm);
      const bool released = !m_limits.hasMaxSwitch || !InterruptHub::isSwitchTriggered(m_limits.maxSwitch);
      if ((nowMs - m_calStateStartMs) >= m_calibration.backoffDurationMs && released) {
        applyDriverPwm(0);
  m_lastAppliedDirection = 0;
        const long minTick = (m_calCapturedMinTick < m_calCapturedMaxTick) ? m_calCapturedMinTick : m_calCapturedMaxTick;
        const long maxTick = (m_calCapturedMinTick > m_calCapturedMaxTick) ? m_calCapturedMinTick : m_calCapturedMaxTick;
        m_limitMinTick = minTick;
        m_limitMaxTick = maxTick;
        m_calCenterTick = minTick + ((maxTick - minTick) / 2);
        // Use closed-loop motion for center approach.
        m_targetTicks = m_calCenterTick;
        resetController();
        m_lastObservedTick = readTicks();
        m_lastMotionMs = nowMs;
        m_state = MotorState::CENTERING;
        m_centerStableStartMs = 0;
      }
      break;
    }

    case MotorState::CENTERING: {
      // Centering strategy:
      // 1) move to geometric center with the same PID+stiction logic as runtime moves
      // 2) once stabilized, re-zero encoder at that physical location.
      // If motion stalls, fall back to a software center-offset so calibration can complete.
      const long current = readTicks();
      const long error = m_targetTicks - current;

      if (absLong(error) <= m_calibration.centerToleranceTicks) {
        applyDriverPwm(0);
  m_lastAppliedDirection = 0;
        if (m_centerStableStartMs == 0) {
          m_centerStableStartMs = nowMs;
        } else if ((nowMs - m_centerStableStartMs) >= m_calibration.centerSettleMs) {
          // Re-zero encoder at center to simplify all future motion commands.
          writeTicks(0);
          m_limitMinTick -= m_calCenterTick;
          m_limitMaxTick -= m_calCenterTick;
          m_limitsKnown = true;
          m_targetTicks = 0;
          m_scanStepActive = false;
          m_scanPwm = 0;
          m_state = MotorState::STICTION_SCAN_POS;
          applyDriverPwm(0);
  m_lastAppliedDirection = 0;
        }
      } else {
        m_centerStableStartMs = 0;
        const int8_t dir = signOf(error);
        if (directionBlockedByLimit(dir)) {
          enterError();
          return;
        }

        if (current != m_lastObservedTick) {
          m_lastObservedTick = current;
          m_lastMotionMs = nowMs;
        } else if ((nowMs - m_lastMotionMs) > m_runtime.noMotionTimeoutMs) {
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.println(F(",CENTERING_STALL,FALLBACK_TO_SOFT_ZERO"));

          // Reliability fallback:
          // If physical centering stalls, we still compute a consistent logical
          // coordinate frame by shifting the encoder so "0" is at the measured
          // geometric center, without forcing more motion.
          const long fallbackCurrent = readTicks();
          writeTicks(fallbackCurrent - m_calCenterTick);
          m_limitMinTick -= m_calCenterTick;
          m_limitMaxTick -= m_calCenterTick;
          m_limitsKnown = true;
          m_targetTicks = readTicks();
          m_scanStepActive = false;
          m_scanPwm = 0;
          m_state = MotorState::STICTION_SCAN_POS;
          applyDriverPwm(0);
          m_lastAppliedDirection = 0;
          m_lastObservedTick = readTicks();
          m_lastMotionMs = nowMs;
          return;
        }

        // Closed-loop centering command (same engine as command-driven motion).
        const float centerDt = static_cast<float>(m_runtime.controlPeriodMs) * 0.001f;
        const int16_t centeredPwm = computeControlPwm(error, centerDt, current, nowMs);
        applyCommandedPwm(centeredPwm, nowMs, current, error);
        m_prevErrorTicks = error;
      }
      break;
    }

    case MotorState::STICTION_SCAN_POS:
      updateStictionScan(nowMs, 1);
      break;

    case MotorState::STICTION_SCAN_NEG:
      updateStictionScan(nowMs, -1);
      break;

    case MotorState::CALIB_RETURN_TO_ZERO: {
      // Final calibration step for RA/DEC:
      // use normal closed-loop motion to converge on logical 0 automatically.
      const long current = readTicks();
      const long error = m_targetTicks - current;  // target is 0

      if (absLong(error) <= 1) {
        applyDriverPwm(0);
        m_lastAppliedDirection = 0;
        if (m_centerStableStartMs == 0) {
          m_centerStableStartMs = nowMs;
        } else if ((nowMs - m_centerStableStartMs) >= m_calibration.centerSettleMs) {
          // Hard-snap encoder to exact 0 once mechanically settled.
          writeTicks(0);
          m_targetTicks = 0;
          forceReadyAtCurrentPosition();
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.println(F(",CALIB_ZERO_LOCKED"));
        }
      } else {
        m_centerStableStartMs = 0;
        const int8_t dir = signOf(error);
        if (directionBlockedByLimit(dir)) {
          enterError();
          return;
        }
        const float dt = static_cast<float>(m_runtime.controlPeriodMs) * 0.001f;
        const int16_t pwm = computeControlPwm(error, dt, current, nowMs);
        applyCommandedPwm(pwm, nowMs, current, error);
        m_prevErrorTicks = error;
      }
      break;
    }

    default:
      break;
  }
}

void MotorBase::startNextStictionStep(unsigned long nowMs, int8_t direction) {
  int16_t nextPwm = 0;
  if (!m_scanStepActive && m_scanPwm == 0) {
    nextPwm = m_calibration.stictionScanStartPwm;
  } else {
    nextPwm = static_cast<int16_t>(m_scanPwm) + static_cast<int16_t>(m_calibration.stictionScanStepPwm);
  }

  if (nextPwm > m_calibration.stictionScanMaxPwm) {
    enterError();
    return;
  }

  if (directionBlockedByLimit(direction)) {
    enterError();
    return;
  }

  m_scanPwm = static_cast<uint8_t>(nextPwm);
  m_scanStartTick = readTicks();
  m_scanStepStartMs = nowMs;
  m_scanStepActive = true;
  driveOpenLoopDirectional(direction, m_scanPwm);
}

void MotorBase::updateStictionScan(unsigned long nowMs, int8_t direction) {
  if (!m_scanStepActive) {
    startNextStictionStep(nowMs, direction);
    return;
  }

  const unsigned long elapsed = nowMs - m_scanStepStartMs;
  if (elapsed < m_calibration.stictionSampleWindowMs) {
    return;
  }

  const long delta = absLong(readTicks() - m_scanStartTick);
  if (delta >= m_calibration.stictionDetectTicks) {
    if (direction > 0) {
      m_stictionMinPwmPos = m_scanPwm;
      m_scanStepActive = false;
      m_scanPwm = 0;
      applyDriverPwm(0);
  m_lastAppliedDirection = 0;
      m_state = MotorState::STICTION_SCAN_NEG;
      return;
    }

    m_stictionMinPwmNeg = m_scanPwm;
    m_adaptiveMinPwmPos = m_stictionMinPwmPos;
    m_adaptiveMinPwmNeg = m_stictionMinPwmNeg;
    m_scanStepActive = false;
    m_scanPwm = 0;
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;

    if (m_calibration.singleSwitchHomeOnly) {
      // Focus: homing ends at switch reference; keep that as ready origin.
      forceReadyAtCurrentPosition();
    } else {
      // RA/DEC: finish calibration by actively moving to logical 0.
      m_targetTicks = 0;
      m_motionMode = MotionMode::POSITION;
      resetController();
      m_centerStableStartMs = 0;
      m_lastObservedTick = readTicks();
      m_lastMotionMs = nowMs;
      m_state = MotorState::CALIB_RETURN_TO_ZERO;
    }
    return;
  }

  startNextStictionStep(nowMs, direction);
}

int16_t MotorBase::computeControlPwm(long errorTicks,
                                     float dtSeconds,
                                     long,
                                     unsigned long) {
  if (errorTicks == 0) {
    return 0;
  }

  const float error = static_cast<float>(errorTicks);
  const float derivative = (error - static_cast<float>(m_prevErrorTicks)) / dtSeconds;
  m_errorIntegral += error * dtSeconds;
  if (m_errorIntegral > m_pid.integralMax) m_errorIntegral = m_pid.integralMax;
  if (m_errorIntegral < m_pid.integralMin) m_errorIntegral = m_pid.integralMin;

  const float pidOut = (m_pid.kp * error) +
                       (m_pid.ki * m_errorIntegral) +
                       (m_pid.kd * derivative);

  const int8_t direction = signOf(errorTicks);
  const int16_t ff = static_cast<int16_t>(readAdaptiveMinPwmForDir(direction));
  int32_t pwm = static_cast<int32_t>(lroundf(pidOut)) + (direction * ff);

  // Immediate stiction compensation: any non-zero error gets at least calibrated minimum PWM.
  if (direction > 0 && pwm < ff) pwm = ff;
  if (direction < 0 && pwm > -ff) pwm = -ff;

  if (pwm > m_runtime.pwmMax) pwm = m_runtime.pwmMax;
  if (pwm < -m_runtime.pwmMax) pwm = -m_runtime.pwmMax;
  return static_cast<int16_t>(pwm);
}

void MotorBase::beginMicroCorrection(unsigned long nowMs, long currentErrorTicks) {
  m_state = MotorState::MICRO_CORRECTION;
  m_microStartMs = nowMs;
  m_microDirection = signOf(currentErrorTicks);
  m_errorIntegral = 0.0f;

  // Overshoot feedback: reduce the adaptive floor in the just-used direction.
  if (m_lastAppliedDirection > 0 && m_adaptiveMinPwmPos > m_stictionMinPwmPos) {
    const uint8_t drop = m_precision.adaptiveDropStep;
    const uint8_t floor = m_stictionMinPwmPos;
    m_adaptiveMinPwmPos = (m_adaptiveMinPwmPos > static_cast<uint8_t>(floor + drop))
                              ? static_cast<uint8_t>(m_adaptiveMinPwmPos - drop)
                              : floor;
  } else if (m_lastAppliedDirection < 0 && m_adaptiveMinPwmNeg > m_stictionMinPwmNeg) {
    const uint8_t drop = m_precision.adaptiveDropStep;
    const uint8_t floor = m_stictionMinPwmNeg;
    m_adaptiveMinPwmNeg = (m_adaptiveMinPwmNeg > static_cast<uint8_t>(floor + drop))
                              ? static_cast<uint8_t>(m_adaptiveMinPwmNeg - drop)
                              : floor;
  }
}

void MotorBase::applyCommandedPwm(int16_t signedPwm,
                                  unsigned long nowMs,
                                  long currentTicks,
                                  long errorTicks) {
  if (signedPwm > m_runtime.pwmMax) signedPwm = m_runtime.pwmMax;
  if (signedPwm < -m_runtime.pwmMax) signedPwm = static_cast<int16_t>(-m_runtime.pwmMax);

  int8_t direction = 0;
  if (signedPwm > 0) direction = 1;
  if (signedPwm < 0) direction = -1;

  if (direction != 0) {
    const uint8_t adaptive = readAdaptiveMinPwmForDir(direction);
    if (direction > 0 && signedPwm < adaptive) signedPwm = adaptive;
    if (direction < 0 && signedPwm > -adaptive) signedPwm = static_cast<int16_t>(-adaptive);
  }

  if (directionBlockedByLimit(direction)) {
    // Hard stop at end-stop and freeze target at current position.
    signedPwm = 0;
    m_targetTicks = currentTicks;
    m_state = MotorState::READY;
    m_motionMode = MotionMode::POSITION;
    resetController();
  }

  applyDriverPwm(signedPwm);
  m_lastAppliedDirection = direction;

  if (direction != 0 && errorTicks != 0) {
    if (currentTicks != m_lastObservedTick) {
      m_lastObservedTick = currentTicks;
      m_lastMotionMs = nowMs;
    } else {
      const unsigned long stuckMs = nowMs - m_lastMotionMs;
      if (stuckMs >= m_runtime.noMotionTimeoutMs) {
        if (absLong(errorTicks) <= 2000) {
          // Near target: try one stronger static-friction kick before faulting.
          const uint8_t raiseStep = 3;
          if (direction > 0) {
            uint16_t raised = static_cast<uint16_t>(m_adaptiveMinPwmPos) + raiseStep;
            if (raised > m_calibration.stictionScanMaxPwm) raised = m_calibration.stictionScanMaxPwm;
            m_adaptiveMinPwmPos = static_cast<uint8_t>(raised);
          } else if (direction < 0) {
            uint16_t raised = static_cast<uint16_t>(m_adaptiveMinPwmNeg) + raiseStep;
            if (raised > m_calibration.stictionScanMaxPwm) raised = m_calibration.stictionScanMaxPwm;
            m_adaptiveMinPwmNeg = static_cast<uint8_t>(raised);
          }
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.print(F(",NEAR_TARGET_STALL_BOOST,err="));
          Serial.println(errorTicks);
          m_lastMotionMs = nowMs;
        } else {
          Serial.print(F("DBG,"));
          Serial.print(m_name);
          Serial.print(F(",STALL_ERROR,err="));
          Serial.println(errorTicks);
          enterError();
          return;
        }
      }

      const uint8_t adaptive = readAdaptiveMinPwmForDir(direction);
      if (stuckMs >= m_precision.adaptiveRaiseDelayMs &&
          absLong(signedPwm) <= static_cast<long>(adaptive + 1)) {
        if (direction > 0) {
          uint16_t raised = static_cast<uint16_t>(m_adaptiveMinPwmPos) + m_precision.adaptiveRaiseStep;
          if (raised > m_calibration.stictionScanMaxPwm) raised = m_calibration.stictionScanMaxPwm;
          m_adaptiveMinPwmPos = static_cast<uint8_t>(raised);
        } else {
          uint16_t raised = static_cast<uint16_t>(m_adaptiveMinPwmNeg) + m_precision.adaptiveRaiseStep;
          if (raised > m_calibration.stictionScanMaxPwm) raised = m_calibration.stictionScanMaxPwm;
          m_adaptiveMinPwmNeg = static_cast<uint8_t>(raised);
        }
        m_lastMotionMs = nowMs;
      }
    }
  } else {
    m_lastObservedTick = currentTicks;
    m_lastMotionMs = nowMs;
  }
}

void MotorBase::updateMicroCorrection(unsigned long nowMs, long currentErrorTicks, long currentTicks) {
  if (currentErrorTicks == 0) {
    forceReadyAtCurrentPosition();
    return;
  }

  const int8_t dir = signOf(currentErrorTicks);
  const uint8_t basePwm = readAdaptiveMinPwmForDir(dir);
  int16_t commanded = static_cast<int16_t>(dir * basePwm);

  const unsigned long elapsed = nowMs - m_microStartMs;
  if (elapsed < m_precision.reverseKickMs) {
    commanded = static_cast<int16_t>(dir * (basePwm + m_precision.reverseKickExtraPwm));
  } else if (absLong(currentErrorTicks) > 1) {
    commanded = static_cast<int16_t>(dir * (basePwm + (m_precision.reverseKickExtraPwm / 2)));
  }

  applyCommandedPwm(commanded, nowMs, currentTicks, currentErrorTicks);
  if (currentErrorTicks == 0) {
    forceReadyAtCurrentPosition();
  }
}

void MotorBase::updateMotion(unsigned long nowMs, float dtSeconds) {
  const long currentTicks = readTicks();
  const long errorTicks = m_targetTicks - currentTicks;

  if (m_state == MotorState::MICRO_CORRECTION) {
    updateMicroCorrection(nowMs, errorTicks, currentTicks);
    m_prevErrorTicks = errorTicks;
    return;
  }

  if (errorTicks == 0) {
    applyCommandedPwm(0, nowMs, currentTicks, errorTicks);
    m_state = MotorState::READY;
    m_prevErrorTicks = 0;
    return;
  }

  // 1-tick precision path:
  // if error sign flips around the target, we switch to MICRO_CORRECTION and
  // apply a short reverse kick plus stiction floor hold until exact tick lock.
  if (m_prevErrorTicks != 0 &&
      signOf(errorTicks) != signOf(m_prevErrorTicks) &&
      absLong(errorTicks) <= m_precision.overshootWindowTicks) {
    beginMicroCorrection(nowMs, errorTicks);
    updateMicroCorrection(nowMs, errorTicks, currentTicks);
    m_prevErrorTicks = errorTicks;
    return;
  }

  int16_t pwm = computeControlPwm(errorTicks, dtSeconds, currentTicks, nowMs);
  applyCommandedPwm(pwm, nowMs, currentTicks, errorTicks);

  if (m_state != MotorState::ERROR && m_state != MotorState::READY) {
    m_state = (m_motionMode == MotionMode::SPEED) ? MotorState::SPEED_MOVING : MotorState::MOVING;
  }
  m_prevErrorTicks = errorTicks;
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
