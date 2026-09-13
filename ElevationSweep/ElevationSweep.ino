// ElevationSweep - bench firmware for one NEMA17 16:1 gearbox axis with an
// AS5048A on the output shaft and an ICM-20948 on the arm.
//
// What it does on boot:
//   1. Brings up display, IMU, encoder and stepper and reports each.
//   2. First run only: jogs a few degrees to learn which way the encoder
//      and IMU count relative to the motor, then jogs back.
//   3. Levels the arm to the horizon using the IMU (assumes the IMU board
//      is mounted square to the arm - flat, on edge or on end - and that
//      the arm starts roughly level) and sets that as 0 deg.
//   4. First run only: jogs +10 deg and asks over serial/OLED whether the
//      arm tip went UP. Answer y/n. That fixes "positive = up" for good.
//   5. Sweeps to 180 deg (the other horizon), dwells, sweeps back to 0.
//   6. Waits for serial commands ('h' lists them).
//
// Everything learned is stored in flash (Preferences) so later boots go
// straight to level + sweep. 'F' wipes it.
//
// Safety: soft limits on every move, ramped speed, and a stall/collision
// trip if the encoder and step count disagree by more than 12 deg.
// Send 'x' at any time to stop. Send 'X' to stop AND cut the driver.

#include <Wire.h>
#include <SPI.h>
#include <Preferences.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SH110X.h>
#include <FastAccelStepper.h>

#include <GearboxAxis_all.h>

#include "config.h"

// ------------------------------------------------------------ hardware

Adafruit_SH1107 display(OLED_W, OLED_H, &Wire, -1, 400000, 100000);
bool displayPresent = false;

ArmIMU imu;

FastAccelStepperEngine engine;

GearboxAxisConfig elCfg = [] {
  GearboxAxisConfig c;
  c.stepPin = PIN_EL_STEP;
  c.dirPin = PIN_EL_DIR;
  c.enablePin = PIN_EL_EN;
  c.csPin = PIN_EL_CS;
  c.gearRatio = GEAR_RATIO;
  c.motorStepsPerRev = MOTOR_STEPS_PER_REV;
  c.microsteps = MICROSTEPS;
  c.maxSpeedDegPerS = MAX_SPEED_DEG_S;
  c.accelDegPerS2 = ACCEL_DEG_S2;
  c.minDeg = SOFT_MIN_DEG;
  c.maxDeg = SOFT_MAX_DEG;
  return c;
}();
GearboxAxis el(elCfg);

Preferences prefs;

// ------------------------------------------------------------ state

enum class Phase {
  Boot,
  Discover,      // learn encoder/IMU signs (first run)
  Level,         // IMU-guided move to the horizon, set zero
  AskUp,         // jog +10 and ask which way the tip went (first run)
  SweepOut,      // 0 -> 180
  Dwell,
  SweepBack,     // 180 -> 0
  Idle,
  Fault,
};
Phase phase = Phase::Boot;
uint32_t phaseStartMs = 0;

struct Cal {
  bool valid = false;         // signs learned
  bool invertMotor = false;
  bool invertSensor = false;
  float imuAxis[3] = {0, 0, 0};
  bool upKnown = false;       // the y/n question has been answered
  bool levelValid = false;    // zeroRaw + imuLevel measured
  float zeroRaw = 0;
  float imuLevel[3] = {0, 0, 0};
  // One-shot action to run on the NEXT boot, which then clears itself. This
  // is how anything long-running gets started: closing a serial session
  // resets this board, so a test begun over serial dies with the session.
  //   0 = nothing, 1 = oscillate, 2 = level then staged sweep,
  //   3 = wave horizon to horizon speeding up each tier,
  //   4 = search for the top-speed ceiling
  int bootAction = 0;
  float oscAmp = 20.0f;
  float sweepTo = 180.0f;
  float sweepStep = 15.0f;
  float waveSpeed = 30.0f;
  int waveCycles = 2;
  float ceilStart = 60.0f;
} cal;

bool rawStream = false;
// Continuous back-and-forth, for watching the axis move and for tuning speed
// and acceleration by eye. Open loop so it cannot be refused by soft limits
// referenced to a stale zero; each leg reports encoder and IMU so a slip
// still shows up.
// The staged sweep's per-station readings are kept in RAM so a sweep that ran
// with nobody connected can still be read out afterwards. Opening a serial
// session does not disturb the board; only closing one resets it.
// The gravity vector is kept alongside the angles because it is what the
// hinge-axis fit needs: the encoder's angle is the truth, and the gravity the
// IMU read there is the observation.
struct Station { float target, enc, imu; float g[3]; };
Station sweepLog[48];
int sweepLogN = 0;

bool oscillating = false;
long oscSteps = 0;
int oscDir = 1;
int oscLegs = 0;
uint32_t oscDwellUntil = 0;
// Hand test: driver off, arm turned by hand, encoder and IMU watched together.
// This is the test that proves the sensing chain without involving the motor.
bool handTest = false;
float handEncMin = NAN, handEncMax = NAN, handImuMin = NAN, handImuMax = NAN;
uint32_t lastReportMs = 0;
uint32_t lastDisplayMs = 0;
const char *statusLine = "boot";

// ------------------------------------------------------------ helpers

void setPhase(Phase p, const char *why) {
  phase = p;
  phaseStartMs = millis();
  statusLine = why;
  Serial.printf("[phase] %s\n", why);
}

void logf(const char *fmt, ...) {
  char buf[160];
  va_list ap;
  va_start(ap, fmt);
  vsnprintf(buf, sizeof(buf), fmt, ap);
  va_end(ap);
  Serial.println(buf);
}

void loadCal() {
  prefs.begin("gbx-el", true);
  cal.valid = prefs.getBool("valid", false);
  cal.invertMotor = prefs.getBool("invM", false);
  cal.invertSensor = prefs.getBool("invS", false);
  prefs.getBytes("axis", cal.imuAxis, sizeof(cal.imuAxis));
  cal.upKnown = prefs.getBool("upKnown", false);
  cal.levelValid = prefs.getBool("lvlValid", false);
  cal.zeroRaw = prefs.getFloat("zeroRaw", 0);
  prefs.getBytes("lvl", cal.imuLevel, sizeof(cal.imuLevel));
  cal.bootAction = prefs.getInt("bootAct", 0);
  cal.oscAmp = prefs.getFloat("oscAmp", 20.0f);
  cal.sweepTo = prefs.getFloat("swTo", 180.0f);
  cal.sweepStep = prefs.getFloat("swStep", 15.0f);
  cal.waveSpeed = prefs.getFloat("wvSpd", 30.0f);
  cal.waveCycles = prefs.getInt("wvCyc", 2);
  cal.ceilStart = prefs.getFloat("ceilSt", 60.0f);
  prefs.end();

  if (PRESET_UP_IS_POSITIVE >= 0 && !cal.upKnown) {
    // Preset means: the positive direction as discovered IS/ISN'T up.
    // We can only apply it once discovery has run; handled in Discover.
  }
}

void saveCal() {
  prefs.begin("gbx-el", false);
  prefs.putBool("valid", cal.valid);
  prefs.putBool("invM", cal.invertMotor);
  prefs.putBool("invS", cal.invertSensor);
  prefs.putBytes("axis", cal.imuAxis, sizeof(cal.imuAxis));
  prefs.putBool("upKnown", cal.upKnown);
  prefs.putBool("lvlValid", cal.levelValid);
  prefs.putFloat("zeroRaw", cal.zeroRaw);
  prefs.putBytes("lvl", cal.imuLevel, sizeof(cal.imuLevel));
  prefs.putInt("bootAct", cal.bootAction);
  prefs.putFloat("oscAmp", cal.oscAmp);
  prefs.putFloat("swTo", cal.sweepTo);
  prefs.putFloat("swStep", cal.sweepStep);
  prefs.putFloat("wvSpd", cal.waveSpeed);
  prefs.putInt("wvCyc", cal.waveCycles);
  prefs.putFloat("ceilSt", cal.ceilStart);
  prefs.end();
}

void wipeCal() {
  prefs.begin("gbx-el", false);
  prefs.clear();
  prefs.end();
  cal = Cal{};
}

void applyCalToHardware() {
  el.setInvertMotor(cal.invertMotor);
  el.setInvertSensor(cal.invertSensor);
  if (cal.valid) imu.setAxis(cal.imuAxis);
  if (cal.levelValid) {
    el.setZeroRawDeg(cal.zeroRaw);
    imu.setLevelReference(cal.imuLevel);
  }
}

// Flip the meaning of "positive" everywhere at once: motor, encoder, IMU.
void flipPositiveDirection() {
  cal.invertMotor = !cal.invertMotor;
  cal.invertSensor = !cal.invertSensor;
  for (int i = 0; i < 3; i++) cal.imuAxis[i] = -cal.imuAxis[i];
  applyCalToHardware();
}

// Pump everything that has to keep running while a blocking step waits.
void service() {
  el.update();
  imu.poll();
  drawDisplay();
}

// Blocking wait for the axis; keeps the display and IMU alive meanwhile.
bool waitAxis(uint32_t timeoutMs = 60000) {
  uint32_t t0 = millis();
  while (el.isBusy()) {
    service();
    if (el.fault()) return false;
    if (millis() - t0 > timeoutMs) return false;
    if (Serial.available()) {
      int c = Serial.peek();
      if (c == 'x' || c == 'X') { Serial.read(); el.emergencyStop(); return false; }
    }
    delay(2);
  }
  return !el.fault();
}

// ------------------------------------------------------------ display

void drawDisplay() {
  if (!displayPresent) return;
  uint32_t now = millis();
  if (now - lastDisplayMs < 200) return;
  lastDisplayMs = now;

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SH110X_WHITE);
  display.setCursor(0, 0);
  display.println("ELEVATION AXIS");
  display.println(statusLine);
  display.println();

  float pos = el.positionDeg();
  float imuDeg = imu.armAngleDeg();
  display.printf("enc %8.2f deg\n", pos);
  if (isnan(imuDeg)) display.println("imu   ---.-- deg");
  else display.printf("imu %8.2f deg\n", imuDeg);
  if (!isnan(el.targetDeg())) display.printf("tgt %8.2f deg\n", el.targetDeg());
  else display.println("tgt      --- ");
  display.printf("steps %ld\n", el.currentSteps());
  display.printf("spd %.0f/s acc %.0f\n", el.config().maxSpeedDegPerS, el.config().accelDegPerS2);
  display.printf("drv %s %s\n", el.enabled() ? "ON " : "off", el.isMoving() ? "MOVING" : "");
  display.printf("enc %s imu %s\n", el.sensorOk() ? "ok" : "ERR", imu.present() ? "ok" : "--");
  if (el.fault()) {
    display.println();
    display.println("** FAULT **");
    display.println(el.faultReason());
  }
  display.display();
}

// ------------------------------------------------------------ reporting

void printStatus() {
  float pos;
  bool ok = el.readPosition(pos);
  AS5048A::Diagnostics d;
  bool dOk = el.sensor().readDiagnostics(d);
  logf("--- status ---");
  logf("phase      : %s", statusLine);
  logf("devices    : display %s  imu %s  encoder %s  stepper %s",
       displayPresent ? "ok" : "MISSING", imu.present() ? "ok" : "MISSING",
       el.sensorPresent() ? "ok" : "MISSING", el.stepsPerDeg() > 0 ? "ok" : "MISSING");
  logf("encoder    : %s  raw %.3f  rel %.3f deg  zero %.3f  errors %lu",
       !el.sensorPresent() ? "ABSENT" : (ok ? "ok" : "FAIL"), el.sensorRawDeg(), pos,
       el.zeroRawDeg(), (unsigned long)el.sensor().errorCount());
  if (dOk)
    logf("magnet     : AGC %u  mag %u  %s%s%s", d.agc, d.magnitude,
         d.compHigh ? "TOO WEAK/FAR " : "", d.compLow ? "TOO STRONG/CLOSE " : "",
         (!d.compHigh && !d.compLow) ? "good" : "");
  logf("imu        : %s  arm %.2f deg  |g| %.2f  gyro %.1f d/s  %s  T %.1fC",
       imu.present() ? "ok" : "absent", imu.armAngleDeg(), imu.gravityMagnitude(),
       imu.gyroRateDegPerS(), imu.stationary() ? "still" : "moving", imu.temperatureC());
  const float *g = imu.gravity();
  logf("imu gravity: %.2f %.2f %.2f   axis %.2f %.2f %.2f", g[0], g[1], g[2],
       cal.imuAxis[0], cal.imuAxis[1], cal.imuAxis[2]);
  logf("stepper    : %s  steps %ld  target %.2f  %s  %.1f deg/s  %.1f deg/s^2  %.1f steps/deg",
       el.enabled() ? "enabled" : "disabled", el.currentSteps(), el.targetDeg(),
       el.isMoving() ? "MOVING" : "stopped", el.config().maxSpeedDegPerS,
       el.config().accelDegPerS2, el.stepsPerDeg());
  logf("cal        : signs %s  invM %d invS %d  upKnown %d  level %s",
       cal.valid ? "learned" : "NOT learned", cal.invertMotor, cal.invertSensor,
       cal.upKnown, cal.levelValid ? "set" : "NOT set");
  logf("driver DIAG: %s", digitalRead(PIN_EL_DIAG) ? "HIGH - driver reports a fault"
                                                   : "low (ok, or pin not wired)");
  if (el.fault()) logf("FAULT      : %s", el.faultReason());
}

void printHelp() {
  Serial.println(
      "Commands:\n"
      "  h        this help\n"
      "  s        status\n"
      "  r        toggle raw stream (encoder / imu / steps at 10 Hz)\n"
      "  B        BEGIN the automatic bring-up (level, then sweep)\n"
      "  t        hand test: driver OFF, turn the arm by hand, watch enc vs imu\n"
      "  m <n>    open-loop move of n microsteps, ignores encoder and limits\n"
      "           (3200 = one motor revolution = 22.5 deg of output shaft)\n"
      "  o <deg>  oscillate back and forth by +-deg until 'x' (default 20)\n"
      "  O <deg>  oscillate on the NEXT boot, with nothing plugged in\n"
      "  S <deg>  staged sweep 0 -> deg -> 0 now, logging enc vs imu\n"
      "  W <deg>  level then staged sweep on the NEXT boot (default 180)\n"
      "  V <d/s>  wave horizon to horizon on the NEXT boot, speeding up\n"
      "  C <d/s>  find the top-speed ceiling on the NEXT boot, from <d/s> up\n"
      "  P        print the last staged sweep's readings\n"
      "  k / K    refit the IMU hinge axis from the last sweep: report / store\n"
      "  g <deg>  go to angle (0 = horizon, 90 = zenith, 180 = far horizon)\n"
      "  j <deg>  jog by delta degrees (negative ok)\n"
      "  v <d/s>  set max speed, deg/s at the output\n"
      "  a <d/s2> set acceleration\n"
      "  x        stop (ramp down, keep holding)\n"
      "  X        emergency stop (driver off)\n"
      "  e        enable driver    d  disable driver (arm may fall!)\n"
      "  z        set current position as 0 (horizon)\n"
      "  l        re-level with the IMU and set 0 there\n"
      "  E <deg>  'the arm is at this elevation RIGHT NOW' (0 horizon, 90 zenith)\n"
      "           the one thing no sensor here can work out for itself\n"
      "  w        run the sweep 0 -> 180 -> 0\n"
      "  c        re-run sign discovery\n"
      "  u        flip which direction is 'up' (all signs together)\n"
      "  y / n    answer the 'did the tip go up?' question\n"
      "  !        clear fault\n"
      "  F        wipe stored calibration and reboot");
}

// ------------------------------------------------------------ phases

// Learn encoder sign and IMU hinge axis from a small open-loop jog.
bool runDiscovery() {
  setPhase(Phase::Discover, "discovering signs");
  if (!el.sensorPresent()) { logf("discovery: no encoder, cannot proceed"); return false; }

  cal.invertMotor = false;
  cal.invertSensor = false;
  applyCalToHardware();

  float g0[3], g1[3];
  bool haveImu = imu.present() && imu.sampleGravity(g0);

  float s0;
  if (!el.readPosition(s0)) return false;

  long steps = lroundf(DISCOVERY_JOG_DEG * el.stepsPerDeg());
  logf("discovery: jogging +%ld steps (%.1f deg at output)", steps, DISCOVERY_JOG_DEG);
  el.moveSteps(steps);
  if (!waitAxis(20000)) return false;
  delay(300);

  float s1;
  if (!el.readPosition(s1)) return false;
  float dS = s1 - s0;
  while (dS > 180) dS -= 360;
  while (dS <= -180) dS += 360;
  logf("discovery: encoder moved %.2f deg (expected about +%.1f)", dS, DISCOVERY_JOG_DEG);
  if (fabsf(dS) < DISCOVERY_JOG_DEG * 0.25f) {
    logf("discovery: encoder barely moved - magnet not on the output shaft, or motor did not turn");
    el.moveSteps(-steps); waitAxis(20000);
    return false;
  }
  cal.invertSensor = dS < 0;

  if (haveImu) {
    if (imu.sampleGravity(g1)) {
      float obs;
      if (imu.learnAxis(g0, g1, obs)) {
        for (int i = 0; i < 3; i++) cal.imuAxis[i] = imu.axis()[i];
        logf("discovery: IMU saw %.2f deg rotation, hinge axis %.2f %.2f %.2f",
             obs, cal.imuAxis[0], cal.imuAxis[1], cal.imuAxis[2]);
      } else {
        logf("discovery: IMU did not see the arm rotate - is it on the arm?");
      }
    }
  } else {
    logf("discovery: no IMU, leveling will be manual (jog then 'z')");
  }

  logf("discovery: returning");
  el.moveSteps(-steps);
  if (!waitAxis(20000)) return false;

  cal.valid = true;
  applyCalToHardware();
  saveCal();
  return true;
}

// Use the IMU to bring the arm to the horizon and set that as zero.
bool runLevel() {
  setPhase(Phase::Level, "leveling with IMU");
  if (!imu.present() || !imu.axisKnown()) {
    logf("level: no IMU axis - jog to the horizon by hand ('j'), then 'z'");
    return false;
  }
  float g[3];
  if (!imu.sampleGravity(g)) return false;

  // A stored reference was measured against the real world, so trust it over
  // re-guessing. Guessing is only for a system that has no reference yet, and
  // it rests on an assumption that is wrong on plenty of mounts: that the
  // board sits square to the arm with one face down when the arm is level.
  // If the guess puts "level" somewhere that is not level, do not fight it -
  // park the arm where you can see its true elevation and use 'E'.
  bool derived = !cal.levelValid;
  if (derived) {
    float off = imu.useNearestBoardAxisAsLevel(g);
    logf("level: no stored reference, GUESSING that the nearest board axis is\n"
         "       down at level (it is %.1f deg from vertical now). If the arm\n"
         "       ends up somewhere that is not the horizon, use 'E' instead.",
         off);
  } else {
    logf("level: using the stored horizon reference");
  }

  for (int iter = 0; iter < 6; iter++) {
    if (!imu.sampleGravity(g)) return false;
    float tilt = imu.armAngleDegFrom(g, 0.0f);   // wrap around 0 here
    if (isnan(tilt)) { logf("level: IMU geometry degenerate"); return false; }
    logf("level: arm is %.2f deg from level", tilt);
    if (fabsf(tilt) > LEVEL_ABORT_DEG) {
      logf("level: too far from level to auto-level safely; jog by hand then 'l'");
      return false;
    }
    if (fabsf(tilt) < LEVEL_DONE_DEG) break;
    // Temporarily widen limits: we do not have a zero yet.
    el.setSoftLimits(-400, 400);
    el.jog(-tilt);
    if (!waitAxis(30000)) { el.setSoftLimits(SOFT_MIN_DEG, SOFT_MAX_DEG); return false; }
    delay(400);
  }
  el.setSoftLimits(SOFT_MIN_DEG, SOFT_MAX_DEG);

  el.setZeroHere();
  // Only re-measure the reference if this run derived it. Re-storing a
  // trusted reference every time would let it walk with each leveling.
  if (derived && imu.sampleGravity(g)) {
    imu.setLevelReference(g);
    for (int i = 0; i < 3; i++) cal.imuLevel[i] = g[i];
  }
  cal.zeroRaw = el.zeroRawDeg();
  cal.levelValid = true;
  saveCal();
  logf("level: zero set at encoder raw %.3f deg", cal.zeroRaw);
  return true;
}

void startAskUp() {
  setPhase(Phase::AskUp, "tip went UP? y/n");
  logf("Jogging +%.0f deg. Did the ARM TIP go UP (away from the ground)?  y / n", ASK_UP_JOG_DEG);
  el.moveTo(ASK_UP_JOG_DEG);
}

void answerAskUp(bool wentUp) {
  if (!wentUp) {
    flipPositiveDirection();
    logf("flipped: positive is now the other way");
    // We are physically at what is now -10 deg; go back to 0.
  } else {
    logf("confirmed: positive = up");
  }
  cal.upKnown = true;
  saveCal();
  el.moveTo(0);
  waitAxis(30000);
  startSweep();
}

void startSweep() {
  setPhase(Phase::SweepOut, "sweep 0 -> 180");
  if (!el.moveTo(SWEEP_TO_DEG)) {
    logf("sweep: move refused");
    setPhase(Phase::Idle, "idle");
  }
}

// Horizon to horizon in stations rather than one long slew, pausing at each
// to let the IMU settle and logging what both sensors say. Two reasons to do
// it this way: the per-station numbers are the linearity and agreement data
// the real tracker needs, and a shorter leg reaches a lower peak speed, so
// anything the arm fouls is met gently and trips the stall guard early.
bool runStagedSweep(float toDeg, float stepDeg) {
  int n = (int)ceilf(fabsf(toDeg) / stepDeg);
  if (n < 1) n = 1;
  logf("staged sweep 0 -> %.0f -> 0 in %d stations of %.1f deg", toDeg, n,
       toDeg / n);
  logf("  %7s %8s %8s %8s", "target", "encoder", "imu", "enc-imu");
  float worst = 0;
  sweepLogN = 0;
  for (int leg = 0; leg < 2; leg++) {
    for (int k = 1; k <= n; k++) {
      // Out: 1/n .. n/n. Back: (n-1)/n .. 0.
      float frac = (leg == 0) ? (float)k / n : (float)(n - k) / n;
      float t = toDeg * frac;
      if (!el.moveTo(t)) {
        logf("  refused at %.2f deg (soft limits %.0f..%.0f, or a fault)", t,
             el.config().minDeg, el.config().maxDeg);
        return false;
      }
      if (!waitAxis(90000)) {
        logf("  ABORTED at %.2f deg: %s", t,
             el.fault() ? el.faultReason() : "timed out or stopped");
        return false;
      }
      uint32_t t0 = millis();
      while (millis() - t0 < 1200) { service(); delay(5); }  // let the IMU settle
      float e = el.positionDeg(), i = imu.armAngleDeg();
      logf("  %7.1f %8.2f %8.2f %8.2f", t, e, i, e - i);
      if (sweepLogN < (int)(sizeof(sweepLog) / sizeof(sweepLog[0]))) {
        Station &st = sweepLog[sweepLogN++];
        st.target = t;
        st.enc = e;
        st.imu = i;
        for (int a = 0; a < 3; a++) st.g[a] = imu.gravity()[a];
      }
      if (!isnan(i) && fabsf(e - i) > worst) worst = fabsf(e - i);
    }
  }
  logf("staged sweep complete. worst encoder-vs-IMU disagreement %.2f deg, %lu "
       "encoder read errors total", worst,
       (unsigned long)el.sensor().errorCount());
  return true;
}

// Continuous horizon to horizon at a chosen speed, closed loop the whole way
// so the stall guard stays armed. Acceleration is set equal to the speed,
// which means "reach full speed in one second" at any speed, so the ramp
// stays proportionate instead of becoming a jerk at the top end.
//
// Reports each leg's wall time against the theoretical time for a trapezoid
// profile. A leg that takes materially longer than predicted means the motor
// is not keeping up with the pulse train it is being given.
bool runWave(float toDeg, int cycles, float speed) {
  const float accel = speed;
  el.setSpeedLimits(speed, accel);
  float ramp = speed / accel;                       // seconds to full speed
  float rampDeg = 0.5f * accel * ramp * ramp;
  float predicted = (2.0f * rampDeg >= fabsf(toDeg))
                        ? 2.0f * sqrtf(fabsf(toDeg) / accel)          // triangular
                        : 2.0f * ramp + (fabsf(toDeg) - 2.0f * rampDeg) / speed;
  logf("wave: %d cycles 0 <-> %.0f at %.0f deg/s, %.0f deg/s^2 (predict %.2fs per leg)",
       cycles, toDeg, speed, accel, predicted);
  for (int c = 0; c < cycles; c++) {
    for (int leg = 0; leg < 2; leg++) {
      float t = (leg == 0) ? toDeg : 0.0f;
      uint32_t t0 = millis();
      if (!el.moveTo(t)) { logf("  refused at %.0f deg", t); return false; }
      if (!waitAxis(120000)) {
        logf("  ABORTED heading for %.0f: %s", t,
             el.fault() ? el.faultReason() : "timed out or stopped");
        return false;
      }
      uint32_t dt = millis() - t0;
      uint32_t s0 = millis();
      while (millis() - s0 < 700) { service(); delay(5); }   // settle the IMU
      logf("  cycle %d -> %5.0f  %5.2fs (%+.2f vs predicted)  enc %7.2f  imu %7.2f", c + 1,
           t, dt / 1000.0f, dt / 1000.0f - predicted, el.positionDeg(), imu.armAngleDeg());
    }
  }
  return true;
}

// Walk the top speed up in 10 deg/s steps until the axis loses sync, and
// report the highest that completed a clean round trip. One number per run,
// so successive belt tensions or a round of grub screws can be compared
// against each other instead of against an anecdote.
//
// Acceleration is set to the speed but capped at 100 deg/s^2, which was
// measured clean at 40 deg/s, so this isolates top speed rather than
// confounding it with acceleration. A tier whose ramp would not fit in half
// the travel is skipped: the axis would never reach that speed, and timing it
// would say nothing.
// Time a ramped move of `dist` degrees should take at this speed and accel.
static float predictedLegSeconds(float dist, float speed, float accel) {
  const float rampDeg = speed * speed / (2.0f * accel);
  if (2.0f * rampDeg >= dist) return 2.0f * sqrtf(dist / accel);   // triangular
  return 2.0f * (speed / accel) + (dist - 2.0f * rampDeg) / speed;
}

bool runCeiling(float startSpeed) {
  float bestClean = NAN, bestCompleted = NAN;
  logf("ceiling search from %.0f deg/s, 10 deg/s steps, over %.0f deg of travel",
       startSpeed, cal.sweepTo);
  logf("  CLEAN = the ramp landed on target unaided. LOSSY = it completed only");
  logf("  because the encoder loop recovered lost steps afterwards.");
  for (float s = startSpeed; s <= 130.0f; s += 10.0f) {
    const float a = fminf(s, 100.0f);
    if (s * s / (2.0f * a) > fabsf(cal.sweepTo) * 0.5f) {
      logf("  %.0f deg/s is unreachable in %.0f deg at accel %.0f - stopping here",
           s, cal.sweepTo, a);
      break;
    }
    const float predict = predictedLegSeconds(fabsf(cal.sweepTo), s, a);
    el.setSpeedLimits(s, a);
    bool ok = true, clean = true;
    for (int leg = 0; leg < 2 && ok; leg++) {
      const float t = (leg == 0) ? cal.sweepTo : 0.0f;
      const uint32_t t0 = millis();
      if (!el.moveTo(t) || !waitAxis(120000)) {
        ok = false;
        break;
      }
      const float dt = (millis() - t0) / 1000.0f;
      const int tries = el.settleTries();
      const uint32_t s0 = millis();
      while (millis() - s0 < 500) { service(); delay(5); }
      // Judge on time, not on the fixup count. A single fixup costing 0.08s is
      // the encoder landing a fraction outside the 0.1 deg settle tolerance,
      // which is normal. Lost steps cost real time to wind back, because the
      // recovery move has its own ramp.
      const float slack = fmaxf(0.20f, 0.05f * predict);
      const bool legClean = (dt - predict <= slack);
      if (!legClean) clean = false;
      logf("  %5.0f deg/s (accel %3.0f) -> %5.0f  %5.2fs (%+.2f vs %.2f predicted)  "
           "%d fixups  enc %7.2f  %s",
           s, a, t, dt, dt - predict, predict, tries, el.positionDeg(),
           legClean ? "CLEAN" : "LOSSY");
    }
    if (!ok) {
      logf("  LOST SYNC at %.0f deg/s, %.2f deg in: %s", s, el.positionDeg(),
           el.fault() ? el.faultReason() : "timed out or stopped");
      break;
    }
    bestCompleted = s;
    if (clean) bestClean = s;
  }
  el.clearFault();
  el.setSpeedLimits(MAX_SPEED_DEG_S, ACCEL_DEG_S2);
  if (el.moveTo(0)) waitAxis(120000);
  if (isnan(bestCompleted)) {
    logf("CEILING: nothing completed, even %.0f deg/s failed", startSpeed);
  } else {
    logf("CEILING: highest CLEAN %.0f deg/s, highest completed-with-recovery %.0f deg/s "
         "(%lu encoder read errors)",
         bestClean, bestCompleted, (unsigned long)el.sensor().errorCount());
    logf("  Use the CLEAN figure. Set the working limit well under it.");
  }
  return true;
}

void runFirstBootSequence() {
  bool ok = true;
  if (!cal.valid) ok = runDiscovery();
  if (ok && !cal.levelValid) ok = runLevel();
  if (!ok) {
    setPhase(Phase::Idle, "manual: see 'h'");
    logf("Automatic setup incomplete. Use 'j' to jog, 'z' to zero, 'w' to sweep.");
    return;
  }
  if (cal.levelValid && cal.valid) {
    // Make sure we are actually at zero (a later boot can find the arm anywhere).
    float pos;
    if (el.readPosition(pos) && fabsf(pos) > 0.5f) {
      logf("boot: arm at %.2f deg, moving to horizon", pos);
      el.moveTo(0);
      if (!waitAxis(60000)) { setPhase(Phase::Idle, "move failed"); return; }
    }
  }
  if (!cal.upKnown) {
    if (PRESET_UP_IS_POSITIVE == 1) { cal.upKnown = true; saveCal(); }
    else if (PRESET_UP_IS_POSITIVE == 0) { flipPositiveDirection(); cal.upKnown = true; saveCal(); }
  }
  if (!cal.upKnown) startAskUp();
  else startSweep();
}

// ------------------------------------------------------------ serial

void handleCommand(const String &line) {
  if (line.length() == 0) return;
  char c = line[0];
  float arg = line.length() > 1 ? line.substring(1).toFloat() : 0;
  switch (c) {
    case 'h': printHelp(); break;
    case 's': printStatus(); break;
    case 'r': rawStream = !rawStream; break;
    case 'B':
      logf("beginning bring-up");
      runFirstBootSequence();
      break;
    case 't':
      handTest = !handTest;
      handEncMin = handEncMax = handImuMin = handImuMax = NAN;
      if (handTest) {
        el.stop();
        el.enable(false);
        setPhase(Phase::Idle, "HAND TEST - driver off");
        logf("Hand test on, driver DISABLED. Turn the arm by hand through a\n"
             "big angle (30 deg or more) and back. Both columns should move\n"
             "together and by the same amount. 't' again to stop.");
      } else {
        logf("Hand test off. encoder swept %.2f deg, imu swept %.2f deg",
             handEncMax - handEncMin, handImuMax - handImuMin);
        float e = handEncMax - handEncMin, i = handImuMax - handImuMin;
        if (!(e > 2.0f) && !(i > 2.0f))
          logf("Neither moved. Did the arm actually turn? Is the magnet on the shaft?");
        else if (!(e > 2.0f))
          logf("The ARM moved but the ENCODER did not: the magnet is not turning with the arm.");
        else if (!(i > 2.0f))
          logf("The ENCODER moved but the IMU did not: the IMU is not riding the arm.");
        else
          logf("Both moved. Ratio imu/enc = %.3f (want about 1.00)", i / e);
      }
      break;
    case 'm': {
      long n = (long)arg;
      if (n == 0) { logf("m needs a step count, e.g. 'm 800'"); break; }
      if (n > 12800 || n < -12800) { logf("refusing more than 4 motor revolutions"); break; }
      logf("open-loop %ld microsteps (%.2f deg of output shaft if geared %.0f:1)", n,
           n / el.stepsPerDeg(), GEAR_RATIO);
      float before = el.positionDeg(), gBefore = imu.armAngleDeg();
      if (!el.moveSteps(n)) { logf("refused (fault? '!' to clear)"); break; }
      setPhase(Phase::Idle, "open-loop move");
      waitAxis(60000);
      delay(400);
      float after; el.readPosition(after);
      imu.poll();
      logf("encoder %.2f -> %.2f (%.2f deg)   imu %.2f -> %.2f (%.2f deg)", before, after,
           after - before, gBefore, imu.armAngleDeg(), imu.armAngleDeg() - gBefore);
      break;
    }
    case 'g':
      if (el.moveTo(arg)) { setPhase(Phase::Idle, "manual move"); logf("go to %.2f", arg); }
      else logf("refused (limits %.0f..%.0f, fault, or no encoder)", SOFT_MIN_DEG, SOFT_MAX_DEG);
      break;
    case 'j':
      if (el.jog(arg)) { setPhase(Phase::Idle, "manual jog"); logf("jog %.2f", arg); }
      else logf("refused");
      break;
    case 'v': if (arg > 0) { el.setSpeedLimits(arg, el.config().accelDegPerS2); logf("max speed %.1f deg/s", arg); } break;
    case 'a': if (arg > 0) { el.setSpeedLimits(el.config().maxSpeedDegPerS, arg); logf("accel %.1f deg/s^2", arg); } break;
    case 'o': {
      float amp = (arg != 0) ? fabsf(arg) : 20.0f;
      oscSteps = lroundf(amp * el.stepsPerDeg());
      oscDir = 1;
      oscLegs = 0;
      oscDwellUntil = 0;
      oscillating = true;
      setPhase(Phase::Idle, "oscillating");
      logf("oscillating +-%.1f deg (%ld microsteps per leg). 'x' to stop.", amp, oscSteps);
      break;
    }
    case 'O': {
      cal.oscAmp = (arg != 0) ? fabsf(arg) : 20.0f;
      cal.bootAction = 1;
      saveCal();
      logf("will oscillate +-%.1f deg on the next boot, then clear the flag.",
           cal.oscAmp);
      break;
    }
    case 'S': {
      float to = (arg != 0) ? arg : cal.sweepTo;
      setPhase(Phase::Idle, "staged sweep");
      runStagedSweep(to, cal.sweepStep);
      setPhase(Phase::Idle, "idle");
      break;
    }
    case 'P': {
      if (sweepLogN == 0) { logf("no sweep recorded since boot"); break; }
      logf("last sweep, %d stations:", sweepLogN);
      logf("  %7s %8s %8s %8s", "target", "encoder", "imu", "enc-imu");
      for (int k = 0; k < sweepLogN; k++)
        logf("  %7.1f %8.2f %8.2f %8.2f", sweepLog[k].target, sweepLog[k].enc,
             sweepLog[k].imu, sweepLog[k].enc - sweepLog[k].imu);
      break;
    }
    // Refit the hinge axis from the last sweep. Lower case reports, upper
    // case adopts and stores, so a bad fit cannot overwrite a working one by
    // accident.
    case 'k':
    case 'K': {
      if (sweepLogN < 6) { logf("need a staged sweep first ('S' or 'W')"); break; }
      static float gs[48][3];
      static float angs[48];
      int n = sweepLogN;
      for (int i = 0; i < n; i++) {
        for (int a = 0; a < 3; a++) gs[i][a] = sweepLog[i].g[a];
        angs[i] = sweepLog[i].enc;    // the encoder is the reference, not the target
      }
      float before = ArmIMU::residualRmsDeg(gs, angs, n, imu.axis(), imu.levelReference());
      float newAxis[3], newLevel[3];
      if (!ArmIMU::fitGeometry(gs, angs, n, imu.axis(), newAxis, newLevel)) {
        logf("fit failed: samples too few or degenerate");
        break;
      }
      float after = ArmIMU::residualRmsDeg(gs, angs, n, newAxis, newLevel);
      logf("hinge axis fit over %d stations:", n);
      logf("  old axis %.4f %.4f %.4f   rms %.3f deg", imu.axis()[0], imu.axis()[1],
           imu.axis()[2], before);
      logf("  new axis %.4f %.4f %.4f   rms %.3f deg", newAxis[0], newAxis[1],
           newAxis[2], after);
      if (c == 'k') {
        logf("  reported only. 'K' to adopt and store it.");
        break;
      }
      if (!(after < before)) {
        logf("  REFUSED: the fit is not better than what is stored.");
        break;
      }
      imu.setAxis(newAxis);
      imu.setLevelReference(newLevel);
      for (int a = 0; a < 3; a++) {
        cal.imuAxis[a] = imu.axis()[a];
        cal.imuLevel[a] = imu.levelReference()[a];
      }
      saveCal();
      logf("  adopted and stored. rms %.3f -> %.3f deg", before, after);
      break;
    }
    case 'V': {
      cal.waveSpeed = (arg != 0) ? fabsf(arg) : 30.0f;
      cal.bootAction = 3;
      saveCal();
      logf("will wave 0 <-> %.0f on the next boot: %d cycles at each of %.0f, %.0f and "
           "%.0f deg/s.",
           cal.sweepTo, cal.waveCycles, cal.waveSpeed, cal.waveSpeed * 1.5f,
           cal.waveSpeed * 2.0f);
      break;
    }
    case 'C': {
      cal.ceilStart = (arg != 0) ? fabsf(arg) : 60.0f;
      cal.bootAction = 4;
      saveCal();
      logf("will search for the speed ceiling from %.0f deg/s on the next boot.",
           cal.ceilStart);
      break;
    }
    case 'W': {
      cal.sweepTo = (arg != 0) ? arg : 180.0f;
      cal.bootAction = 2;
      saveCal();
      logf("will level then staged-sweep 0 -> %.0f -> 0 on the next boot.",
           cal.sweepTo);
      break;
    }
    case 'x':
      oscillating = false;
      if (cal.bootAction != 0) { cal.bootAction = 0; saveCal(); }
      el.stop();
      setPhase(Phase::Idle, "stopped");
      break;
    case 'X': oscillating = false; el.emergencyStop(); setPhase(Phase::Idle, "E-STOP"); break;
    case 'e': el.enable(true); logf("driver enabled"); break;
    case 'd': el.enable(false); logf("driver disabled - arm is free"); break;
    case 'z': {
      el.setZeroHere();
      cal.zeroRaw = el.zeroRawDeg();
      float g[3];
      if (imu.present() && imu.axisKnown() && imu.sampleGravity(g)) {
        imu.setLevelReference(g);
        for (int i = 0; i < 3; i++) cal.imuLevel[i] = g[i];
      }
      cal.levelValid = true;
      saveCal();
      logf("zero set here (raw %.3f)", cal.zeroRaw);
      break;
    }
    case 'E': {
      // The one fact no sensor on this machine can work out for itself: how
      // the IMU board sits on the arm. Park the arm where you can see its
      // true elevation, say so once, and every reference follows from it.
      if (!imu.present() || !imu.axisKnown()) {
        logf("E needs the IMU hinge axis. Run 'c' first.");
        break;
      }
      float g[3];
      if (!imu.sampleGravity(g)) { logf("E: IMU read failed"); break; }
      if (!imu.setLevelFromElevation(g, arg)) { logf("E: could not set reference"); break; }
      el.setPositionHere(arg);
      cal.zeroRaw = el.zeroRawDeg();
      for (int i = 0; i < 3; i++) cal.imuLevel[i] = imu.levelReference()[i];
      cal.levelValid = true;
      saveCal();
      for (int i = 0; i < 40; i++) { imu.poll(); delay(5); }   // settle the filter
      float pos;
      el.readPosition(pos);
      logf("recorded: the arm is at %.2f deg elevation now.\n"
           "  encoder zero raw %.3f, encoder reads %.2f, imu reads %.2f",
           arg, cal.zeroRaw, pos, imu.armAngleDeg());
      break;
    }
    case 'l': runLevel(); setPhase(Phase::Idle, "leveled"); break;
    case 'w': startSweep(); break;
    case 'c': runDiscovery(); setPhase(Phase::Idle, "discovery done"); break;
    case 'u': flipPositiveDirection(); saveCal(); logf("flipped positive direction; re-zero with 'l' or 'z'"); break;
    // During the AskUp phase these answer the question and carry on into the
    // sweep. Outside it they simply record the answer, for when you already
    // know which way is up and want to stop being asked.
    case 'y':
      if (phase == Phase::AskUp) { answerAskUp(true); break; }
      cal.upKnown = true;
      saveCal();
      logf("recorded: positive angles are UP");
      break;
    case 'n':
      if (phase == Phase::AskUp) { answerAskUp(false); break; }
      flipPositiveDirection();
      cal.upKnown = true;
      saveCal();
      logf("flipped: positive is now the other way. Re-level with 'l' before moving.");
      break;
    case '!': el.clearFault(); setPhase(Phase::Idle, "fault cleared"); break;
    case 'F': wipeCal(); logf("calibration wiped, rebooting"); delay(200); ESP.restart(); break;
    default: logf("unknown '%c' - 'h' for help", c);
  }
}

void pollSerial() {
  static String line;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\r') continue;
    if (c == '\n') { handleCommand(line); line = ""; }
    else if (line.length() < 32) line += c;
  }
}

// ------------------------------------------------------------ setup/loop

void setup() {
  Serial.begin(115200);
  uint32_t t0 = millis();
  while (!Serial && millis() - t0 < 3000) delay(10);
  Serial.println("\n\nElevationSweep - NEMA17 16:1 + AS5048A + ICM-20948");

  // STEMMA QT rail is behind a load switch; nothing on I2C answers until this is high.
  pinMode(PIN_I2C_PWR, OUTPUT);
  digitalWrite(PIN_I2C_PWR, HIGH);
  delay(100);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(400000);

  displayPresent = display.begin(OLED_I2C_ADDR, true);
  logf("display : %s", displayPresent ? "SH1107 ok" : "not found");
  if (displayPresent) {
    display.setRotation(OLED_ROTATION);
    display.clearDisplay();
    display.setTextSize(1);
    display.setTextColor(SH110X_WHITE);
    display.setCursor(0, 0);
    display.println("ElevationSweep");
    display.println("booting...");
    display.display();
  }

  logf("imu     : %s", imu.begin(Wire, IMU_I2C_ADDR) ? "ICM-20948 ok" : "not found");

  // Pulled down so an unwired DIAG pin reads "no fault" rather than floating.
  pinMode(PIN_EL_DIAG, INPUT_PULLDOWN);

  SPI.begin(PIN_SPI_SCK, PIN_SPI_MISO, PIN_SPI_MOSI);

  engine.init();
  if (!el.begin(engine)) {
    logf("stepper : FAILED to attach STEP pin %d", PIN_EL_STEP);
  } else {
    logf("stepper : ok  STEP %d DIR %d EN %d  %.1f steps/deg  max %.1f deg/s",
         PIN_EL_STEP, PIN_EL_DIR, PIN_EL_EN, el.stepsPerDeg(), el.config().maxSpeedDegPerS);
  }
  if (el.sensorPresent()) {
    AS5048A::Diagnostics d;
    el.sensor().readDiagnostics(d);
    logf("encoder : AS5048A ok  raw %.3f deg  AGC %u  mag %u%s%s", el.sensorRawDeg(), d.agc,
         d.magnitude, d.compHigh ? "  MAGNET TOO FAR" : "", d.compLow ? "  MAGNET TOO CLOSE" : "");
  } else {
    logf("encoder : AS5048A NOT answering on CS %d (check 3V3+5V tied, MISO, mode)", PIN_EL_CS);
  }

  loadCal();
  applyCalToHardware();
  logf("cal     : signs %s, level %s, up %s", cal.valid ? "stored" : "unknown",
       cal.levelValid ? "stored" : "unknown", cal.upKnown ? "known" : "unknown");

  printHelp();

  // Seed the IMU filter before anyone asks it a question.
  for (int i = 0; i < 20; i++) { imu.poll(); delay(10); }

  if (el.sensorPresent() && cal.bootAction != 0) {
    // Clear the request in flash FIRST, so a reset part-way through does not
    // leave the board repeating this every time it powers on.
    int action = cal.bootAction;
    cal.bootAction = 0;
    saveCal();
    if (action == 1) {
      oscSteps = lroundf(cal.oscAmp * el.stepsPerDeg());
      oscDir = 1;
      oscLegs = 0;
      oscDwellUntil = 0;
      oscillating = true;
      setPhase(Phase::Idle, "oscillating (boot demo)");
      logf("\n*** OSCILLATING +-%.1f deg now, no USB session needed. ***\n"
           "Watch three places: the motor pulley, the gearbox output shaft, and\n"
           "the arm tip. Power-cycle or press RESET to stop.", cal.oscAmp);
    } else if (action == 2) {
      logf("\n*** LEVEL + STAGED SWEEP to %.0f deg, no USB session needed. ***\n"
           "Press RESET or cut power to stop it.", cal.sweepTo);
      setPhase(Phase::Idle, "boot sweep: leveling");
      if (runLevel()) {
        setPhase(Phase::Idle, "boot sweep");
        runStagedSweep(cal.sweepTo, cal.sweepStep);
      }
      setPhase(Phase::Idle, "idle after boot sweep");
    } else if (action == 3) {
      logf("\n*** WAVE: horizon to horizon, getting faster each tier. ***\n"
           "No USB session needed. Press RESET or cut power to stop it.");
      setPhase(Phase::Idle, "wave");
      if (el.moveTo(0)) waitAxis(120000);
      for (int tier = 0; tier < 3 && !el.fault(); tier++) {
        if (!runWave(cal.sweepTo, cal.waveCycles, cal.waveSpeed * (1.0f + 0.5f * tier)))
          break;
      }
      // Leave the axis on the gentle defaults, parked level.
      el.setSpeedLimits(MAX_SPEED_DEG_S, ACCEL_DEG_S2);
      if (!el.fault() && el.moveTo(0)) waitAxis(120000);
      logf("wave finished. %lu encoder read errors total.",
           (unsigned long)el.sensor().errorCount());
      setPhase(Phase::Idle, "idle after wave");
    } else if (action == 4) {
      logf("\n*** CEILING SEARCH from %.0f deg/s. No USB session needed. ***\n"
           "Press RESET or cut power to stop it.", cal.ceilStart);
      setPhase(Phase::Idle, "ceiling search");
      if (el.moveTo(0)) waitAxis(120000);
      runCeiling(cal.ceilStart);
      setPhase(Phase::Idle, "idle after ceiling search");
    }
  } else if (!el.sensorPresent()) {
    setPhase(Phase::Idle, "no encoder");
  } else if (SAFE_BOOT) {
    el.enable(false);
    setPhase(Phase::Idle, "SAFE BOOT - nothing will move");
    logf("\nSAFE_BOOT is on: the driver is disabled and nothing moves on its own.\n"
         "  't' hand test (prove the encoder and IMU follow the arm)\n"
         "  'm 800' open-loop motor test\n"
         "  'B' run the automatic level-and-sweep bring-up\n"
         "Set SAFE_BOOT to false in config.h once you trust the axis.");
  } else {
    runFirstBootSequence();
  }
}

void loop() {
  service();
  pollSerial();

  uint32_t now = millis();

  if (el.fault() && phase != Phase::Fault) {
    setPhase(Phase::Fault, el.faultReason());
    logf("FAULT: %s  ('!' to clear)", el.faultReason());
  }

  switch (phase) {
    case Phase::SweepOut:
      if (!el.isBusy()) {
        logf("sweep: reached %.2f (encoder), imu says %.2f", el.positionDeg(), imu.armAngleDeg());
        setPhase(Phase::Dwell, "dwell at 180");
      }
      break;
    case Phase::Dwell:
      if (now - phaseStartMs >= SWEEP_DWELL_MS) {
        setPhase(Phase::SweepBack, "sweep 180 -> 0");
        el.moveTo(0);
      }
      break;
    case Phase::SweepBack:
      if (!el.isBusy()) {
        logf("sweep: back at %.2f (encoder), imu says %.2f", el.positionDeg(), imu.armAngleDeg());
        setPhase(Phase::Idle, "idle - 'w' to sweep");
      }
      break;
    default:
      break;
  }

  if (oscillating && !el.isBusy()) {
    if (oscDwellUntil == 0) {
      oscDwellUntil = now + 500;   // brief pause at each end
    } else if (now >= oscDwellUntil) {
      oscDwellUntil = 0;
      oscLegs++;
      logf("osc leg %d: enc %.2f  imu %.2f  -> %s%.1f deg", oscLegs, el.positionDeg(),
           imu.armAngleDeg(), oscDir > 0 ? "+" : "-", oscSteps / el.stepsPerDeg());
      el.moveSteps(oscDir * oscSteps);
      oscDir = -oscDir;
    }
  }

  if (handTest && now - lastReportMs >= 200) {
    lastReportMs = now;
    float e = el.positionDeg(), i = imu.armAngleDeg();
    if (!isnan(e)) {
      if (isnan(handEncMin) || e < handEncMin) handEncMin = e;
      if (isnan(handEncMax) || e > handEncMax) handEncMax = e;
    }
    if (!isnan(i)) {
      if (isnan(handImuMin) || i < handImuMin) handImuMin = i;
      if (isnan(handImuMax) || i > handImuMax) handImuMax = i;
    }
    logf("hand  enc %8.2f (span %6.2f)   imu %8.2f (span %6.2f)", e,
         handEncMax - handEncMin, i, handImuMax - handImuMin);
    return;
  }

  if (rawStream && now - lastReportMs >= 100) {
    lastReportMs = now;
    logf("enc %8.3f  imu %8.2f  steps %8ld  tgt %7.2f  %s", el.positionDeg(), imu.armAngleDeg(),
         el.currentSteps(), el.targetDeg(), el.isMoving() ? "moving" : "");
  } else if (!rawStream && el.isMoving() && now - lastReportMs >= 500) {
    lastReportMs = now;
    logf("  enc %7.2f  imu %7.2f  -> %7.2f", el.positionDeg(), imu.armAngleDeg(), el.targetDeg());
  }
}
