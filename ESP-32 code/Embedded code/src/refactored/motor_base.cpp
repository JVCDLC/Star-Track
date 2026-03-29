#include "motor_base.hpp"

#if defined(STARTRACK_ENABLE_REFACTORED_ARCH)

#include <math.h>

namespace refactored {

/**
 * @brief Constructor for MotorBase.
 * Initializes all member variables with provided configuration and default values.
 * Sets up initial state as uninitialized, with stiction floors starting at calibration minimum.
 */
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
      m_motionMode(MotionMode::POSITION),          // Default to position control
      m_state(MotorState::UNINITIALIZED),          // Start uninitialized
      m_limitsKnown(false),                        // Limits not yet calibrated
      m_limitMinTick(0),
      m_limitMaxTick(0),
      m_targetTicks(0),
      m_errorIntegral(0.0f),                       // PID integral starts at zero
      m_prevErrorTicks(0),
      m_lastControlMs(0),
      m_lastMotionMs(0),
      m_lastObservedTick(0),
      m_lastAppliedDirection(0),
      // Initialize stiction floors to calibration start value
      m_stictionMinPwmPos(calibration.stictionScanStartPwm),
      m_stictionMinPwmNeg(calibration.stictionScanStartPwm),
      m_adaptiveMinPwmPos(calibration.stictionScanStartPwm),
      m_adaptiveMinPwmNeg(calibration.stictionScanStartPwm),
      // Calibration scratch variables
      m_calCapturedMinTick(0),
      m_calCapturedMaxTick(0),
      m_calFirstHitTick(0),
      m_calCenterTick(0),
      m_calStateStartMs(0),
      m_releaseStableStartMs(0),
      m_centerStableStartMs(0),
      // Stiction scan state
      m_scanStepActive(false),
      m_scanPwm(0),
      m_scanStartTick(0),
      m_scanStepStartMs(0),
      // Micro-correction state
      m_microStartMs(0),
      m_microDirection(0) {}

/**
 * @brief Returns the sign of a long integer value.
 * @param value The value to check
 * @return 1 if positive, -1 if negative, 0 if zero
 */
int8_t MotorBase::signOf(long value) {
  if (value > 0) return 1;
  if (value < 0) return -1;
  return 0;
}

/**
 * @brief Returns the absolute value of a long integer.
 * @param value The value to take absolute value of
 * @return Absolute value as long
 */
long MotorBase::absLong(long value) {
  return (value < 0) ? -value : value;
}

/**
 * @brief Initialize the motor hardware.
 * Sets up driver pins, zeros PWM output, and sets initial state to stopped.
 * Records current position as target to prevent unwanted movement.
 */
void MotorBase::begin() {
  setupDriverPins();                    // Configure hardware pins (virtual call)
  applyDriverPwm(0);                    // Ensure motor is stopped
  m_lastAppliedDirection = 0;           // No direction applied yet
  m_targetTicks = readTicks();          // Set target to current position
  m_lastObservedTick = m_targetTicks;   // Record starting position
  m_lastControlMs = millis();           // Record startup time
  m_lastMotionMs = m_lastControlMs;     // Initialize motion timestamp
  m_state = MotorState::STOPPED;        // Ready but not moving
}

/**
 * @brief Request calibration to be performed.
 * Resets all calibration-related state and begins the calibration state machine.
 * This will interrupt any current operation and start from the beginning.
 */
void MotorBase::requestCalibration() {
  resetController();                           // Clear PID state
  m_limitsKnown = false;                       // Forget previous limits
  m_state = MotorState::BACKOFF_FROM_LIMIT;   // Start calibration state machine
  m_motionMode = MotionMode::POSITION;         // Use position control for calibration
  m_calStateStartMs = millis();                // Record start time
  // Reset calibration timing variables
  m_releaseStableStartMs = 0;
  m_centerStableStartMs = 0;
  m_scanStepActive = false;                    // Not scanning stiction yet
  m_scanPwm = 0;
  m_calFirstHitTick = readTicks();            // Record position when calibration started
  m_microDirection = 0;                        // Clear micro-correction state
  applyDriverPwm(0);                           // Stop motor
  m_lastAppliedDirection = 0;
}

/**
 * @brief Command the motor to move to an absolute position.
 * @param targetTicks The target position in encoder ticks
 * @return true if command accepted, false if rejected (error state or out of limits)
 * 
 * Validates the command, interrupts calibration if necessary, sets the target,
 * and transitions to appropriate state (READY if already at target, MOVING otherwise).
 */
bool MotorBase::commandPositionTicks(long targetTicks) {
  if (isError()) {
    return false;  // Reject commands if motor is in error state
  }
  if (!targetInsideKnownLimits(targetTicks)) {
    return false;  // Reject if target is outside calibrated limits
  }

  // Allow command-driven interruption of calibration to support per-axis testing.
  if (isCalibrating()) {
    applyDriverPwm(0);        // Stop any calibration movement
    m_lastAppliedDirection = 0;
    m_scanStepActive = false; // Cancel any active stiction scan
  }

  m_targetTicks = targetTicks;      // Set new target position
  m_motionMode = MotionMode::POSITION;  // Use position control mode
  resetController();                // Clear PID state for fresh start

  if (absLong(readTicks() - m_targetTicks) <= static_cast<long>(m_runtime.atTargetToleranceTicks)) {
    // Already at target - go to ready state
    m_state = MotorState::READY;
    applyDriverPwm(0);
    m_lastAppliedDirection = 0;
  } else {
    // Need to move - enter moving state
    m_state = MotorState::MOVING;
  }
  return true;
}

/**
 * @brief Default implementation of speed control command (not supported in base class).
 * @param targetTicks Final target position (unused)
 * @param ticksPerSecond Desired speed (unused)
 * @return false - base class doesn't support speed control
 * 
 * This is overridden by MD13SMotor which implements speed control.
 */
bool MotorBase::commandSpeedTicksPerSecond(long, float) {
  return false;  // Not supported in base class
}

/**
 * @brief Update position-loop PID gains at runtime.
 * @param kp New proportional gain
 * @param ki New integral gain
 * @param kd New derivative gain
 * @return true if gains are valid and applied
 */
bool MotorBase::setPositionPidGains(float kp, float ki, float kd) {
  if (!isfinite(kp) || !isfinite(ki) || !isfinite(kd)) {
    return false;
  }
  if (kp < 0.0f || ki < 0.0f || kd < 0.0f) {
    return false;
  }
  m_pid.kp = kp;
  m_pid.ki = ki;
  m_pid.kd = kd;
  resetController();
  return true;
}

/**
 * @brief Emergency stop - hold current position with maximum PWM.
 * Sets target to current position and applies maximum holding force.
 */
void MotorBase::emergencyStopAndHold() {
  const long nowPos = readTicks();     // Get current position
  m_targetTicks = nowPos;              // Set target to current position
  m_motionMode = MotionMode::POSITION; // Use position control
  m_state = MotorState::READY;         // Ready but holding
  resetController();                   // Clear PID state
  applyDriverPwm(0);                   // Stop movement (will be overridden by holding logic)
  m_lastAppliedDirection = 0;
}

/**
 * @brief Read current encoder position.
 * @return Current position in encoder ticks
 */
long MotorBase::readTicks() const {
  return InterruptHub::readEncoderTicks(m_encoderId);
}

/**
 * @brief Force-set the encoder position counter.
 * @param ticks New position value
 * 
 * Used during calibration to zero the encoder at known positions.
 */
void MotorBase::writeTicks(long ticks) {
  InterruptHub::writeEncoderTicks(m_encoderId, ticks);
}

/**
 * @brief Check if motor is currently in any calibration state.
 * @return true if calibrating, false otherwise
 */
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

/**
 * @brief Reset PID controller state.
 * Clears integral term, previous error, and motion-related variables.
 */
void MotorBase::resetController() {
  m_errorIntegral = 0.0f;      // Clear accumulated integral
  m_prevErrorTicks = 0;        // Clear derivative history
  m_lastAppliedDirection = 0;  // Clear direction history
  m_microDirection = 0;        // Clear micro-correction state
}

/**
 * @brief Force the motor to ready state at current position.
 * Used to recover from errors or calibration interruptions.
 */
void MotorBase::forceReadyAtCurrentPosition() {
  m_targetTicks = readTicks();          // Set target to current position
  m_motionMode = MotionMode::POSITION;  // Position control mode
  m_state = MotorState::READY;          // Ready state
  resetController();                    // Clear PID state
  applyDriverPwm(0);                    // Stop motor
  m_lastAppliedDirection = 0;
}

/**
 * @brief Get the adaptive stiction floor for a given direction.
 * @param direction Motor direction (positive or negative)
 * @return Minimum PWM value required for motion in that direction
 */
uint8_t MotorBase::readAdaptiveMinPwmForDir(int8_t direction) const {
  if (direction >= 0) {
    return m_adaptiveMinPwmPos;  // Positive direction floor
  }
  return m_adaptiveMinPwmNeg;    // Negative direction floor
}

/**
 * @brief Check if movement in a direction is blocked by limit switches.
 * @param direction Intended direction of movement
 * @return true if the direction is blocked by a triggered limit switch
 */
bool MotorBase::directionBlockedByLimit(int8_t direction) const {
  if (direction == 0) {
    return false;  // No movement, not blocked
  }
  // Check if trying to move toward min limit and min switch is triggered
  if (m_limits.hasMinSwitch &&
      direction == m_limits.dirTowardMin &&
      InterruptHub::isSwitchTriggered(m_limits.minSwitch)) {
    return true;
  }
  // Check if trying to move toward max limit and max switch is triggered
  if (m_limits.hasMaxSwitch &&
      direction == m_limits.dirTowardMax &&
      InterruptHub::isSwitchTriggered(m_limits.maxSwitch)) {
    return true;
  }
  return false;  // Direction is clear
}

/**
 * @brief Check if any configured limit switch is currently triggered.
 * @return true if any limit switch is active
 */
bool MotorBase::anyConfiguredLimitTriggered() const {
  const bool minHit = m_limits.hasMinSwitch && InterruptHub::isSwitchTriggered(m_limits.minSwitch);
  const bool maxHit = m_limits.hasMaxSwitch && InterruptHub::isSwitchTriggered(m_limits.maxSwitch);
  return minHit || maxHit;
}

/**
 * @brief Check if target position is within calibrated limits.
 * @param targetTicks Position to check
 * @return true if within limits or limits unknown, false if outside known limits
 */
bool MotorBase::targetInsideKnownLimits(long targetTicks) const {
  if (!m_limitsKnown) {
    return true;  // If limits not calibrated, allow any target
  }
  return targetTicks >= m_limitMinTick && targetTicks <= m_limitMaxTick;
}

/**
 * @brief Enter error state and stop all motion.
 * Used when calibration fails or other unrecoverable errors occur.
 */
void MotorBase::enterError() {
  m_state = MotorState::ERROR;          // Set error state
  m_targetTicks = readTicks();          // Set target to current position
  m_motionMode = MotionMode::POSITION;  // Position control mode
  resetController();                    // Clear PID state
  applyDriverPwm(0);                    // Stop motor
  m_lastAppliedDirection = 0;
}

/**
 * @brief Drive motor in open-loop directional control.
 * @param direction Direction to move (-1, 0, +1)
 * @param pwmMagnitude PWM value for movement (0-255)
 * 
 * Applies PWM in specified direction, but checks for limit switch blocking.
 * Used during calibration phases.
 */
void MotorBase::driveOpenLoopDirectional(int8_t direction, uint8_t pwmMagnitude) {
  // Enforce per-motor runtime PWM cap in open-loop states (calibration, backoff, etc.).
  if (pwmMagnitude > m_runtime.pwmMax) {
    pwmMagnitude = m_runtime.pwmMax;
  }

  if (direction == 0 || pwmMagnitude == 0) {
    applyDriverPwm(0);      // Stop if no direction or no magnitude
    m_lastAppliedDirection = 0;
    return;
  }
  if (directionBlockedByLimit(direction)) {
    applyDriverPwm(0);      // Stop if direction blocked by limit
    m_lastAppliedDirection = 0;
    return;
  }
  applyDriverPwm(static_cast<int16_t>(direction * pwmMagnitude));  // Apply directional PWM
  m_lastAppliedDirection = direction;
}

/**
 * @brief Main update function - called regularly (every 5ms) to control motor.
 * @param nowMs Current timestamp in milliseconds
 * 
 * This is the core control loop that:
 * 1. Enforces safety limits on every call
 * 2. Runs PID control at specified intervals
 * 3. Advances calibration state machine if calibrating
 * 4. Handles motion control and error recovery
 * 
 * Must be called frequently but is designed to be non-blocking.
 */
void MotorBase::update(unsigned long nowMs) {
  // Fast safety layer: Check limits on every call, not just at control period
  // This provides immediate response to limit switches
  if (m_lastAppliedDirection != 0 && directionBlockedByLimit(m_lastAppliedDirection)) {
    applyDriverPwm(0);                    // Emergency stop
    m_lastAppliedDirection = 0;
    if (!isCalibrating()) {
      m_targetTicks = readTicks();        // Update target to current position
      m_motionMode = MotionMode::POSITION;
      m_state = MotorState::READY;        // Go to ready state
      resetController();
    }
  }

  // Check if it's time for control update (every controlPeriodMs)
  const unsigned long elapsedMs = nowMs - m_lastControlMs;  // Safe rollover subtraction
  if (elapsedMs < m_runtime.controlPeriodMs) {
    return;  // Not time yet, exit early
  }
  m_lastControlMs = nowMs;  // Update last control time
  const float dtSeconds = (elapsedMs > 0) ? (elapsedMs * 0.001f) : 0.001f;  // Convert to seconds

  // Handle calibration state machine
  if (isCalibrating()) {
    updateCalibration(nowMs);
    return;
  }

  // Handle non-active states
  if (m_state == MotorState::UNINITIALIZED || m_state == MotorState::ERROR || m_state == MotorState::STOPPED) {
    applyDriverPwm(0);  // Ensure motor is stopped
    return;
  }

  // Active motion control
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

          // Focus single-switch flow ends here by design:
          // 1) hit min at fast speed
          // 2) back off
          // 3) re-hit min slowly
          // 4) stop and stay on switch as home (tick 0)
          if (m_calibration.singleSwitchHomeOnly) {
            writeTicks(0);
            m_limitMinTick = 0;
            m_limitMaxTick = m_limits.fallbackSpanTicks;
            m_limitsKnown = true;
            m_targetTicks = 0;
            forceReadyAtCurrentPosition();
            Serial.print(F("DBG,"));
            Serial.print(m_name);
            Serial.print(F(",HOME_DONE,range=["));
            Serial.print(m_limitMinTick);
            Serial.print(F(","));
            Serial.print(m_limitMaxTick);
            Serial.println(F("]"));
          } else {
            m_calStateStartMs = nowMs;
            m_state = MotorState::BACKOFF_AFTER_MIN_VERIFY;
            applyDriverPwm(0);
            m_lastAppliedDirection = 0;
          }
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
          // Final homing pass:
          // 1) approach home at slow verification speed
          // 2) require stable switch activation to avoid edge bounce
          // 3) define software travel window [0 .. fallbackSpanTicks]
          // 4) stay parked at home (tick 0)
          if (InterruptHub::isSwitchTriggered(m_limits.minSwitch)) {
            if (m_releaseStableStartMs == 0) {
              m_releaseStableStartMs = nowMs;
              applyDriverPwm(0);
              m_lastAppliedDirection = 0;
            } else if ((nowMs - m_releaseStableStartMs) >= m_calibration.releaseStableMs) {
              m_releaseStableStartMs = 0;
              writeTicks(0);
              m_limitMinTick = 0;
              m_limitMaxTick = m_limits.fallbackSpanTicks;
              m_limitsKnown = true;  // Enforce hard software window after homing.
              m_targetTicks = 0;
              forceReadyAtCurrentPosition();
              Serial.print(F("DBG,"));
              Serial.print(m_name);
              Serial.print(F(",HOME_DONE,range=["));
              Serial.print(m_limitMinTick);
              Serial.print(F(","));
              Serial.print(m_limitMaxTick);
              Serial.println(F("]"));
            }
          } else {
            m_releaseStableStartMs = 0;
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

/**
 * @brief Compute PID control output for position control.
 * @param errorTicks Position error (target - current)
 * @param dtSeconds Time delta since last update
 * @param currentTicks Current encoder position (unused in base implementation)
 * @param nowMs Current timestamp (unused in base implementation)
 * @return Signed PWM value (-255 to +255) to apply to motor
 * 
 * Implements standard PID control with:
 * - Proportional term: responds to current error
 * - Integral term: responds to accumulated error over time
 * - Derivative term: responds to rate of error change
 * - Feed-forward stiction compensation
 * - Anti-windup on integral term
 * - Output clamping to PWM limits
 */
int16_t MotorBase::computeControlPwm(long errorTicks,
                                     float dtSeconds,
                                     long,
                                     unsigned long) {
  if (errorTicks == 0) {
    return 0;  // No error, no output
  }

  const float error = static_cast<float>(errorTicks);
  
  // Calculate derivative term (rate of error change)
  const float derivative = (error - static_cast<float>(m_prevErrorTicks)) / dtSeconds;
  
  // Accumulate integral term
  m_errorIntegral += error * dtSeconds;
  
  // Anti-windup: clamp integral to prevent runaway
  if (m_errorIntegral > m_pid.integralMax) m_errorIntegral = m_pid.integralMax;
  if (m_errorIntegral < m_pid.integralMin) m_errorIntegral = m_pid.integralMin;

  // Compute PID output
  const float pidOut = (m_pid.kp * error) +                    // Proportional
                       (m_pid.ki * m_errorIntegral) +          // Integral
                       (m_pid.kd * derivative);                // Derivative

  // Determine movement direction
  const int8_t direction = signOf(errorTicks);
  
  // Add feed-forward stiction compensation
  const int16_t ff = static_cast<int16_t>(readAdaptiveMinPwmForDir(direction));
  int32_t pwm = static_cast<int32_t>(lroundf(pidOut)) + (direction * ff);

  // Ensure minimum PWM for stiction compensation (immediate correction)
  if (direction > 0 && pwm < ff) pwm = ff;
  if (direction < 0 && pwm > -ff) pwm = -ff;

  // Clamp to maximum PWM limits
  if (pwm > m_runtime.pwmMax) pwm = m_runtime.pwmMax;
  if (pwm < -m_runtime.pwmMax) pwm = -m_runtime.pwmMax;
  return static_cast<int16_t>(pwm);
}

/**
 * @brief Begin micro-correction sequence for precision positioning.
 * @param nowMs Current timestamp
 * @param currentErrorTicks Current position error
 * 
 * Initiates precision positioning when motor overshoots target.
 * Applies reverse kick to absorb momentum, then holds with stiction floor.
 * Also reduces adaptive stiction floor as feedback that current floor is too high.
 */
void MotorBase::beginMicroCorrection(unsigned long nowMs, long currentErrorTicks) {
  m_state = MotorState::MICRO_CORRECTION;    // Enter micro-correction state
  m_microStartMs = nowMs;                     // Record start time
  m_microDirection = signOf(currentErrorTicks); // Direction of error
  m_errorIntegral = 0.0f;                     // Reset PID integral

  // Adaptive learning: reduce stiction floor since we overshot
  // This indicates the current floor is too high for precise control
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
    // In SPEED mode, a short reverse pulse can appear from speed-loop transients.
    // If that reverse direction is blocked by a pressed end-stop but the desired
    // target direction is safe, recover by forcing a minimum forward command
    // instead of prematurely finishing the move.
    const int8_t desiredDir = signOf(errorTicks);
    if (m_motionMode == MotionMode::SPEED &&
        desiredDir != 0 &&
        desiredDir != direction &&
        !directionBlockedByLimit(desiredDir)) {
      const uint8_t floor = readAdaptiveMinPwmForDir(desiredDir);
      signedPwm = static_cast<int16_t>(desiredDir * floor);
      direction = desiredDir;
    } else {
      // Hard stop at end-stop and freeze target at current position.
      signedPwm = 0;
      m_targetTicks = currentTicks;
      m_state = MotorState::READY;
      m_motionMode = MotionMode::POSITION;
      resetController();
      direction = 0;
    }
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

/**
 * @brief Update motion control using PID algorithm.
 * @param nowMs Current timestamp
 * @param dtSeconds Time delta since last control update
 * 
 * This method implements the core motion control logic:
 * 1. Calculate position error (target - current)
 * 2. Handle micro-correction for precision positioning
 * 3. Check if target reached
 * 4. Detect overshoot for micro-correction triggering
 * 5. Compute PID PWM output
 * 6. Apply PWM with safety checks
 * 7. Update motor state
 */
void MotorBase::updateMotion(unsigned long nowMs, float dtSeconds) {
  const long currentTicks = readTicks();              // Get current encoder position
  const long errorTicks = m_targetTicks - currentTicks; // Calculate position error

  // Handle micro-correction state (precision positioning)
  if (m_state == MotorState::MICRO_CORRECTION) {
    updateMicroCorrection(nowMs, errorTicks, currentTicks);
    m_prevErrorTicks = errorTicks;
    return;
  }

  // Check if target reached within configured tolerance.
  // RA/DEC keep strict behavior with tolerance=0, while focus can
  // use a small deadband (e.g. 1-2 ticks) to prevent end-of-move hunting.
  const long settleTol = static_cast<long>(m_runtime.atTargetToleranceTicks);
  if (absLong(errorTicks) <= settleTol) {
    applyCommandedPwm(0, nowMs, currentTicks, errorTicks);  // Stop motor
    m_state = MotorState::READY;                             // Go to ready state
    onTargetReached(nowMs, currentTicks);
    m_prevErrorTicks = 0;
    return;
  }

  // Precision positioning: detect overshoot and trigger micro-correction
  // If error sign changed (overshot target) and within precision window, start micro-correction
  if (m_prevErrorTicks != 0 &&
      signOf(errorTicks) != signOf(m_prevErrorTicks) &&
      absLong(errorTicks) <= m_precision.overshootWindowTicks) {
    beginMicroCorrection(nowMs, errorTicks);                 // Start micro-correction
    updateMicroCorrection(nowMs, errorTicks, currentTicks);
    m_prevErrorTicks = errorTicks;
    return;
  }

  // Standard PID control
  int16_t pwm = computeControlPwm(errorTicks, dtSeconds, currentTicks, nowMs);
  applyCommandedPwm(pwm, nowMs, currentTicks, errorTicks);

  // Update state based on motion mode
  if (m_state != MotorState::ERROR && m_state != MotorState::READY) {
    m_state = (m_motionMode == MotionMode::SPEED) ? MotorState::SPEED_MOVING : MotorState::MOVING;
  }
  m_prevErrorTicks = errorTicks;  // Store for next iteration's overshoot detection
}

}  // namespace refactored

#endif  // STARTRACK_ENABLE_REFACTORED_ARCH
