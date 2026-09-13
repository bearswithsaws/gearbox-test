// One geared stepper axis with an absolute encoder on the OUTPUT shaft.
//
// The motor is driven open loop through a STEP/DIR driver (A4988 or any
// compatible) using FastAccelStepper, which generates the pulses in
// hardware so ramps stay smooth no matter how busy loop() gets. The
// AS5048A on the output shaft is the source of truth for where the arm is:
// every move is planned as "steps needed to get from where the encoder
// says we are to where we want to be", and after the ramp finishes the
// residual error is measured and corrected in one or two small moves.
//
// While a move is in flight the step count is compared with the encoder.
// If they disagree by more than stallToleranceDeg the axis is stopped and
// disabled and a fault is raised - that is what a stalled motor, a slipped
// belt, or an arm that hit something looks like.
//
// Angles are in degrees at the output shaft, relative to a zero you set
// (setZeroHere), positive in whatever direction invertMotor/invertSensor
// make positive. Soft limits are enforced on every move request.
#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <FastAccelStepper.h>

#include "AS5048A.h"

struct GearboxAxisConfig {
  int stepPin = -1;
  int dirPin = -1;
  int enablePin = -1;          // -1 if the driver's EN is tied off
  int csPin = -1;              // AS5048A chip select

  float gearRatio = 16.0f;     // output turns : motor turns
  int motorStepsPerRev = 200;  // 1.8 deg motor
  int microsteps = 16;         // as set on the driver's MS pins

  bool invertMotor = false;    // true if positive angle needs DIR low
  bool invertSensor = false;   // true if the encoder counts against the motor

  float maxSpeedDegPerS = 15.0f;   // at the output shaft
  float accelDegPerS2 = 8.0f;

  float minDeg = -5.0f;        // soft limits, relative to zero
  float maxDeg = 185.0f;
  float wrapCenterDeg = 90.0f; // encoder angle unwrapped into (c-180, c+180]

  float settleToleranceDeg = 0.1f;  // "close enough" after a move
  int maxSettleTries = 4;
  float stallToleranceDeg = 12.0f;  // steps-vs-encoder disagreement => fault
};

class GearboxAxis {
 public:
  GearboxAxis(const GearboxAxisConfig &cfg, SPIClass &spi = SPI);

  // engine.init() must already have been called. Returns false if the
  // stepper could not be attached; the encoder result is reported by
  // sensorPresent() so the axis can still be driven open loop on a bench.
  bool begin(FastAccelStepperEngine &engine);

  const GearboxAxisConfig &config() const { return cfg_; }
  float stepsPerDeg() const { return stepsPerDeg_; }

  // ---- encoder ----
  bool sensorPresent() const { return sensorPresent_; }
  bool sensorOk() const { return sensorOk_; }           // recent reads good
  float sensorRawDeg();                                  // 0..360 from chip, NAN on failure
  bool readPosition(float &deg);                         // fresh read, zero+sign applied
  float positionDeg() const { return lastPos_; }         // last good reading
  AS5048A &sensor() { return sensor_; }

  void setZeroHere() { setPositionHere(0.0f); }
  // Declare the axis to be at `deg` right now, shifting the zero to suit.
  // setZeroHere() is just this with 0.
  void setPositionHere(float deg);
  void setZeroRawDeg(float raw) { zeroRaw_ = raw; }
  float zeroRawDeg() const { return zeroRaw_; }

  void setInvertMotor(bool v) { cfg_.invertMotor = v; }
  void setInvertSensor(bool v) { cfg_.invertSensor = v; }
  void setSpeedLimits(float maxDegPerS, float accelDegPerS2);
  void setSoftLimits(float minDeg, float maxDeg) { cfg_.minDeg = minDeg; cfg_.maxDeg = maxDeg; }

  // ---- motion (non-blocking; call update() from loop) ----
  bool moveTo(float deg);          // false if refused (limits, fault, no encoder)
  bool jog(float deltaDeg);
  bool moveSteps(long steps);      // open loop, no encoder involved, still ramped
  void stop();                     // ramp down and hold
  void emergencyStop();            // stop now and disable the driver
  bool isMoving() const;
  bool isBusy() const { return isMoving() || settling_; }
  float targetDeg() const { return targetDeg_; }
  bool hasTarget() const { return settling_ || isMoving(); }
  long currentSteps() const;
  // Where the step count says we are, given the encoder reading when the
  // current move started. Only meaningful during a move.
  float expectedDegFromSteps() const;

  void enable(bool on);
  bool enabled() const { return enabled_; }

  void update();

  bool fault() const { return fault_; }
  const char *faultReason() const { return faultReason_; }
  void clearFault();

  // Blocking helper: pump update() until the current move and its settle
  // corrections are done, or timeoutMs passes. Returns false on timeout
  // or fault.
  bool waitUntilSettled(uint32_t timeoutMs = 60000);

 private:
  float wrapRelative(float deg) const;
  bool planMoveFrom(float fromDeg, float toDeg);
  void raiseFault(const char *why);

  GearboxAxisConfig cfg_;
  AS5048A sensor_;
  FastAccelStepper *stepper_ = nullptr;
  float stepsPerDeg_ = 0;

  bool sensorPresent_ = false;
  bool sensorOk_ = false;
  int sensorFailures_ = 0;
  float zeroRaw_ = 0;
  float lastPos_ = NAN;

  bool enabled_ = false;
  bool fault_ = false;
  const char *faultReason_ = "";

  bool settling_ = false;
  bool openLoopMove_ = false;
  int settleTries_ = 0;
  float targetDeg_ = NAN;
  long stepsAtStart_ = 0;
  float degAtStart_ = 0;
};
