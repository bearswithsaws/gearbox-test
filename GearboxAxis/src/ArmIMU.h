// ICM-20948 riding an arm: turns the gravity vector into "how far has the
// arm rotated about its hinge", without needing to know how the IMU is
// bolted on.
//
// Geometry is learned, not configured. Jog the arm a little in the motor's
// positive direction and hand this class the averaged gravity vector from
// before and after; the hinge axis is the normal of the plane those two
// vectors span, signed so that a positive motor move reads as a positive
// angle. After that, armAngleDeg() is the signed rotation of the current
// gravity vector away from a stored "level" gravity vector, measured about
// that axis.
//
// The only assumption made anywhere is in useNearestBoardAxisAsLevel():
// that the IMU board is mounted square to the arm (flat, on edge, or on
// end - any of the six ways), so the arm is level when whichever board
// axis is currently closest to vertical is exactly vertical. That is used
// once to find the horizon on a fresh system, with the arm placed roughly
// level; after that the true measured level vector is stored instead.
#pragma once

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_ICM20948.h>

class ArmIMU {
 public:
  bool begin(TwoWire &wire = Wire, uint8_t addr = 0x69);
  bool present() const { return present_; }

  // Read one sample and fold it into the low-pass filtered gravity estimate.
  // Call from loop(). Returns false if the read failed.
  bool poll();

  // Blocking average of `samples` readings, for calibration moments where
  // the arm is known to be still. m/s^2, body frame.
  bool sampleGravity(float g[3], int samples = 40, int delayMs = 5);

  // Filtered gravity from poll(), body frame, m/s^2.
  const float *gravity() const { return gFilt_; }
  float gravityMagnitude() const;
  // True when the arm is holding still (|g| near 9.81 and the gyro quiet).
  bool stationary() const { return stationary_; }
  float gyroRateDegPerS() const { return gyroDeg_; }
  float temperatureC() const { return tempC_; }

  // Learn the hinge axis from gravity before/after a POSITIVE motor jog.
  // Fails if the two vectors are nearly parallel (arm did not move, or the
  // IMU is not on the arm). Returns the observed rotation in degrees.
  bool learnAxis(const float gBefore[3], const float gAfter[3], float &observedDeg);
  bool axisKnown() const { return axisKnown_; }
  void setAxis(const float a[3]);
  const float *axis() const { return axis_; }
  // If the motor direction is later inverted, the axis sign must follow.
  void flipAxis();

  // Level reference. useNearestBoardAxisAsLevel() picks the one of +-X,
  // +-Y, +-Z closest to the gravity vector given (the arm is expected to be
  // roughly level when this is called) and returns how far off it was.
  // setLevelReference() stores a measured vector instead.
  //
  // Both of those only work if the board happens to be mounted square to the
  // arm with a known face down at level. setLevelFromElevation() is the
  // honest version and needs no such assumption: tell it the arm's true
  // elevation right now (0 = horizon, 90 = zenith) and it back-rotates the
  // measured gravity about the hinge axis to derive the level reference. One
  // operator statement calibrates any mounting, at any angle.
  float useNearestBoardAxisAsLevel(const float gNow[3]);
  bool setLevelFromElevation(const float gNow[3], float elevationDeg);
  void setLevelReference(const float g[3]);
  bool levelKnown() const { return levelKnown_; }
  const float *levelReference() const { return level_; }

  // Signed arm angle in degrees from the level reference, positive in the
  // motor-positive sense, unwrapped into (wrapCenter-180, wrapCenter+180].
  // NAN until the axis and level reference are known.
  float armAngleDeg(float wrapCenterDeg = 90.0f) const;
  float armAngleDegFrom(const float g[3], float wrapCenterDeg = 90.0f) const;

  // Same computation against an explicit geometry, so a candidate axis can be
  // scored without adopting it.
  static float angleFrom(const float g[3], const float axis[3],
                         const float level[3], float wrapCenterDeg = 90.0f);

  // Fit the hinge axis and level reference from samples taken across a real
  // sweep: the encoder's angle (which is accurate) paired with the gravity
  // vector the IMU read there.
  //
  // Rotation about the hinge cannot change gravity's component along the
  // hinge, so the measured gravity vectors trace a circle in the plane normal
  // to the axis. The axis is therefore that plane's normal, recovered by
  // least squares over every sample. That uses the whole sweep as its
  // baseline instead of one short discovery jog, which is what makes it
  // sharp. axisHint only picks the sign, so the rotation sense still matches
  // the motor. Nothing is stored: pass the results to setAxis() and
  // setLevelReference() to adopt them.
  static bool fitGeometry(const float (*g)[3], const float *angleDeg, int n,
                          const float axisHint[3], float axisOut[3],
                          float levelOut[3]);

  // RMS of (sample angle - angle predicted by this geometry), in degrees.
  static float residualRmsDeg(const float (*g)[3], const float *angleDeg, int n,
                              const float axis[3], const float level[3]);

 private:
  Adafruit_ICM20948 icm_;
  bool present_ = false;
  bool axisKnown_ = false;
  bool levelKnown_ = false;
  float axis_[3] = {0, 0, 0};
  float level_[3] = {0, 0, 0};
  float gFilt_[3] = {0, 0, 0};
  bool filtSeeded_ = false;
  bool stationary_ = false;
  float gyroDeg_ = 0;
  float tempC_ = 0;
};
