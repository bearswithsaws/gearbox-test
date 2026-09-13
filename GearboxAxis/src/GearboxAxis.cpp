#include "GearboxAxis.h"

#include <math.h>

GearboxAxis::GearboxAxis(const GearboxAxisConfig &cfg, SPIClass &spi)
    : cfg_(cfg), sensor_(cfg.csPin, spi) {
  stepsPerDeg_ = (float)cfg_.motorStepsPerRev * cfg_.microsteps * cfg_.gearRatio / 360.0f;
}

bool GearboxAxis::begin(FastAccelStepperEngine &engine) {
  stepper_ = engine.stepperConnectToPin(cfg_.stepPin);
  if (!stepper_) return false;
  stepper_->setDirectionPin(cfg_.dirPin);
  if (cfg_.enablePin >= 0) {
    stepper_->setEnablePin(cfg_.enablePin, true);   // EN is active low on A4988/DRV8825/TMC2209 alike
    stepper_->setAutoEnable(false);                 // hold torque stays on
  }
  setSpeedLimits(cfg_.maxSpeedDegPerS, cfg_.accelDegPerS2);
  enable(false);

  sensorPresent_ = sensor_.begin();
  sensorOk_ = sensorPresent_;
  if (sensorPresent_) {
    float d;
    readPosition(d);
  }
  return true;
}

void GearboxAxis::setSpeedLimits(float maxDegPerS, float accelDegPerS2) {
  cfg_.maxSpeedDegPerS = maxDegPerS;
  cfg_.accelDegPerS2 = accelDegPerS2;
  if (!stepper_) return;
  uint32_t hz = (uint32_t)lroundf(maxDegPerS * stepsPerDeg_);
  if (hz < 1) hz = 1;
  stepper_->setSpeedInHz(hz);
  int32_t acc = (int32_t)lroundf(accelDegPerS2 * stepsPerDeg_);
  if (acc < 1) acc = 1;
  stepper_->setAcceleration(acc);
  // FastAccelStepper applies new speed/accel on the next move call; if a
  // move is in flight, applySpeedAcceleration() makes it take effect now.
  stepper_->applySpeedAcceleration();
}

// ---------------------------------------------------------------- encoder

float GearboxAxis::wrapRelative(float deg) const {
  float c = cfg_.wrapCenterDeg;
  while (deg > c + 180.0f) deg -= 360.0f;
  while (deg <= c - 180.0f) deg += 360.0f;
  return deg;
}

float GearboxAxis::sensorRawDeg() { return sensor_.readAngleDeg(); }

bool GearboxAxis::readPosition(float &deg) {
  float raw = sensor_.readAngleDeg();
  if (isnan(raw)) {
    if (sensorFailures_ < 100) sensorFailures_++;
    if (sensorFailures_ >= 5) sensorOk_ = false;
    return false;
  }
  sensorFailures_ = 0;
  sensorOk_ = true;
  float rel = raw - zeroRaw_;
  if (cfg_.invertSensor) rel = -rel;
  deg = wrapRelative(rel);
  lastPos_ = deg;
  return true;
}

void GearboxAxis::setPositionHere(float deg) {
  float raw = sensor_.readAngleDeg();
  if (isnan(raw)) return;
  // readPosition computes pos = wrap(+-(raw - zero)), so invert that for zero.
  zeroRaw_ = cfg_.invertSensor ? (raw + deg) : (raw - deg);
  while (zeroRaw_ < 0.0f) zeroRaw_ += 360.0f;
  while (zeroRaw_ >= 360.0f) zeroRaw_ -= 360.0f;
  float d;
  readPosition(d);
}

// ----------------------------------------------------------------- motion

void GearboxAxis::enable(bool on) {
  if (!stepper_) return;
  if (on) stepper_->enableOutputs(); else stepper_->disableOutputs();
  enabled_ = on;
}

bool GearboxAxis::isMoving() const { return stepper_ && stepper_->isRunning(); }

long GearboxAxis::currentSteps() const { return stepper_ ? stepper_->getCurrentPosition() : 0; }

float GearboxAxis::expectedDegFromSteps() const {
  long delta = currentSteps() - stepsAtStart_;
  float deg = (float)delta / stepsPerDeg_;
  if (cfg_.invertMotor) deg = -deg;
  return degAtStart_ + deg;
}

bool GearboxAxis::planMoveFrom(float fromDeg, float toDeg) {
  float deltaDeg = toDeg - fromDeg;
  long deltaSteps = lroundf(deltaDeg * stepsPerDeg_);
  if (cfg_.invertMotor) deltaSteps = -deltaSteps;
  stepsAtStart_ = currentSteps();
  degAtStart_ = fromDeg;
  if (deltaSteps == 0) return true;
  return stepper_->move((int32_t)deltaSteps) == MOVE_OK;
}

bool GearboxAxis::moveTo(float deg) {
  if (!stepper_ || fault_) return false;
  if (deg < cfg_.minDeg || deg > cfg_.maxDeg) return false;

  float from;
  if (isMoving()) {
    // Re-targeting mid-move: the encoder lags the motor through the belt,
    // so plan from the step count instead and let settle() fix the rest.
    from = expectedDegFromSteps();
    stepper_->stopMove();
    while (stepper_->isRunning()) { delay(1); }
    from = expectedDegFromSteps();
  } else if (!readPosition(from)) {
    return false;   // no encoder, no closed-loop move
  }
  if (!enabled_) enable(true);

  targetDeg_ = deg;
  settling_ = true;
  openLoopMove_ = false;
  settleTries_ = 0;
  return planMoveFrom(from, deg);
}

bool GearboxAxis::jog(float deltaDeg) {
  float from;
  if (isMoving()) from = expectedDegFromSteps();
  else if (!readPosition(from)) return false;
  return moveTo(from + deltaDeg);
}

bool GearboxAxis::moveSteps(long steps) {
  if (!stepper_ || fault_) return false;
  if (!enabled_) enable(true);
  float from;
  if (!readPosition(from)) from = NAN;
  stepsAtStart_ = currentSteps();
  degAtStart_ = from;
  targetDeg_ = NAN;
  settling_ = false;
  openLoopMove_ = true;
  return stepper_->move((int32_t)steps) == MOVE_OK;
}

void GearboxAxis::stop() {
  if (!stepper_) return;
  stepper_->stopMove();
  settling_ = false;
  targetDeg_ = NAN;
}

void GearboxAxis::emergencyStop() {
  if (!stepper_) return;
  stepper_->forceStop();
  settling_ = false;
  targetDeg_ = NAN;
  enable(false);
}

void GearboxAxis::raiseFault(const char *why) {
  fault_ = true;
  faultReason_ = why;
  emergencyStop();
}

void GearboxAxis::clearFault() {
  fault_ = false;
  faultReason_ = "";
}

void GearboxAxis::update() {
  if (!stepper_) return;

  float pos;
  bool fresh = readPosition(pos);

  if (isMoving()) {
    if (fresh && !openLoopMove_ && !isnan(degAtStart_)) {
      float expected = expectedDegFromSteps();
      if (fabsf(expected - pos) > cfg_.stallToleranceDeg) {
        raiseFault("encoder disagrees with step count (stall/slip/collision)");
      }
    }
    return;
  }

  if (openLoopMove_) {
    openLoopMove_ = false;
    return;
  }

  if (!settling_) return;

  if (!fresh) {
    if (!sensorOk_) {
      settling_ = false;
      raiseFault("encoder stopped answering during settle");
    }
    return;
  }

  float err = targetDeg_ - pos;
  if (fabsf(err) <= cfg_.settleToleranceDeg || settleTries_ >= cfg_.maxSettleTries) {
    settling_ = false;   // done: either on target or as close as we get
    return;
  }
  settleTries_++;
  planMoveFrom(pos, targetDeg_);
}

bool GearboxAxis::waitUntilSettled(uint32_t timeoutMs) {
  uint32_t t0 = millis();
  while (isBusy()) {
    update();
    if (fault_) return false;
    if (millis() - t0 > timeoutMs) return false;
    delay(2);
  }
  return !fault_;
}
