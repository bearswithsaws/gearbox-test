#include "ArmIMU.h"

#include <math.h>

namespace {
constexpr float R2D = 180.0f / (float)M_PI;

float dot(const float a[3], const float b[3]) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
void cross(const float a[3], const float b[3], float out[3]) {
  out[0] = a[1] * b[2] - a[2] * b[1];
  out[1] = a[2] * b[0] - a[0] * b[2];
  out[2] = a[0] * b[1] - a[1] * b[0];
}
float norm(const float a[3]) { return sqrtf(dot(a, a)); }
bool normalize(const float a[3], float out[3]) {
  float n = norm(a);
  if (n < 1e-6f) return false;
  out[0] = a[0] / n; out[1] = a[1] / n; out[2] = a[2] / n;
  return true;
}
// Remove the component of v along unit axis a.
void projectOntoPlane(const float v[3], const float a[3], float out[3]) {
  float d = dot(v, a);
  out[0] = v[0] - d * a[0];
  out[1] = v[1] - d * a[1];
  out[2] = v[2] - d * a[2];
}
float wrapAround(float deg, float center) {
  while (deg > center + 180.0f) deg -= 360.0f;
  while (deg <= center - 180.0f) deg += 360.0f;
  return deg;
}
}  // namespace

bool ArmIMU::begin(TwoWire &wire, uint8_t addr) {
  present_ = icm_.begin_I2C(addr, &wire);
  if (!present_) return false;
  icm_.setAccelRange(ICM20948_ACCEL_RANGE_4_G);
  icm_.setGyroRange(ICM20948_GYRO_RANGE_500_DPS);
  // ~100 Hz output; the arm moves at a few deg/s so this is plenty.
  icm_.setAccelRateDivisor(10);
  icm_.setGyroRateDivisor(10);
  return true;
}

bool ArmIMU::poll() {
  if (!present_) return false;
  sensors_event_t a, g, t, m;
  if (!icm_.getEvent(&a, &g, &t, &m)) return false;
  float raw[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
  tempC_ = t.temperature;
  gyroDeg_ = sqrtf(g.gyro.x * g.gyro.x + g.gyro.y * g.gyro.y + g.gyro.z * g.gyro.z) * R2D;

  // A dead bus reads all zeros; do not let that pollute the filter.
  float mag = norm(raw);
  if (mag < 0.5f) return false;

  if (!filtSeeded_) {
    for (int i = 0; i < 3; i++) gFilt_[i] = raw[i];
    filtSeeded_ = true;
  } else {
    constexpr float A = 0.1f;
    for (int i = 0; i < 3; i++) gFilt_[i] += A * (raw[i] - gFilt_[i]);
  }
  stationary_ = fabsf(mag - 9.81f) < 1.0f && gyroDeg_ < 3.0f;
  return true;
}

float ArmIMU::gravityMagnitude() const { return norm(gFilt_); }

bool ArmIMU::sampleGravity(float g[3], int samples, int delayMs) {
  if (!present_) return false;
  double acc[3] = {0, 0, 0};
  int good = 0;
  for (int i = 0; i < samples; i++) {
    sensors_event_t a, gy, t, m;
    if (icm_.getEvent(&a, &gy, &t, &m)) {
      float raw[3] = {a.acceleration.x, a.acceleration.y, a.acceleration.z};
      if (norm(raw) > 0.5f) {
        for (int k = 0; k < 3; k++) acc[k] += raw[k];
        good++;
      }
    }
    delay(delayMs);
  }
  if (good < samples / 2) return false;
  for (int k = 0; k < 3; k++) g[k] = (float)(acc[k] / good);
  return true;
}

bool ArmIMU::learnAxis(const float gBefore[3], const float gAfter[3], float &observedDeg) {
  float b[3], a[3];
  if (!normalize(gBefore, b) || !normalize(gAfter, a)) return false;
  // Rotating the body by +theta about axis n rotates gravity (seen in the
  // body frame) by -theta, so gAfter x gBefore points along +n.
  float c[3];
  cross(a, b, c);
  float s = norm(c);
  float cth = dot(a, b);
  observedDeg = atan2f(s, cth) * R2D;
  if (observedDeg < 0.5f) return false;   // too small to trust the direction
  if (!normalize(c, axis_)) return false;
  axisKnown_ = true;
  return true;
}

void ArmIMU::setAxis(const float a[3]) {
  axisKnown_ = normalize(a, axis_);
}

void ArmIMU::flipAxis() {
  for (int i = 0; i < 3; i++) axis_[i] = -axis_[i];
}

float ArmIMU::useNearestBoardAxisAsLevel(const float gNow[3]) {
  float g[3];
  if (!normalize(gNow, g)) return NAN;
  int best = 0;
  for (int i = 1; i < 3; i++) {
    if (fabsf(g[i]) > fabsf(g[best])) best = i;
  }
  float sign = (g[best] >= 0) ? 1.0f : -1.0f;
  level_[0] = level_[1] = level_[2] = 0;
  level_[best] = sign * 9.81f;
  levelKnown_ = true;
  return acosf(fminf(1.0f, fabsf(g[best]))) * R2D;
}

// Rodrigues: rotate v about unit axis n by phi degrees.
static void rotateAbout(const float v[3], const float n[3], float phiDeg, float out[3]) {
  float phi = phiDeg * (float)M_PI / 180.0f;
  float c = cosf(phi), s = sinf(phi);
  float nxv[3];
  cross(n, v, nxv);
  float ndotv = dot(n, v);
  for (int i = 0; i < 3; i++)
    out[i] = v[i] * c + nxv[i] * s + n[i] * ndotv * (1.0f - c);
}

bool ArmIMU::setLevelFromElevation(const float gNow[3], float elevationDeg) {
  if (!axisKnown_) return false;
  // Body rotation of +d about the axis rotates body-frame gravity by -d, so
  // stepping from the current elevation down to 0 rotates gravity by
  // +elevation. armAngleDegFrom() then reads back exactly elevationDeg here.
  float g[3];
  rotateAbout(gNow, axis_, elevationDeg, g);
  setLevelReference(g);
  return levelKnown_;
}

void ArmIMU::setLevelReference(const float g[3]) {
  for (int i = 0; i < 3; i++) level_[i] = g[i];
  levelKnown_ = norm(level_) > 0.5f;
}

float ArmIMU::angleFrom(const float g[3], const float axis[3], const float level[3],
                        float wrapCenterDeg) {
  float gp[3], lp[3];
  projectOntoPlane(g, axis, gp);
  projectOntoPlane(level, axis, lp);
  if (norm(gp) < 0.5f || norm(lp) < 0.5f) return NAN;   // hinge axis vertical?
  // Body rotated +theta from level => gravity rotated -theta => the signed
  // angle from g to level, about the axis, is +theta.
  float c[3];
  cross(gp, lp, c);
  float theta = atan2f(dot(axis, c), dot(gp, lp)) * R2D;
  return wrapAround(theta, wrapCenterDeg);
}

float ArmIMU::armAngleDegFrom(const float g[3], float wrapCenterDeg) const {
  if (!axisKnown_ || !levelKnown_) return NAN;
  return angleFrom(g, axis_, level_, wrapCenterDeg);
}

// Eigenvector of the smallest eigenvalue of a symmetric 3x3, row major.
// Analytic rather than iterative: three values from the characteristic cubic,
// then the null space of (C - lambda I) as the longest cross product of its
// row pairs, which stays well conditioned when two eigenvalues are close.
static bool smallestEigenvector(const double C[9], float out[3]) {
  const double p1 = C[1] * C[1] + C[2] * C[2] + C[5] * C[5];
  const double q = (C[0] + C[4] + C[8]) / 3.0;
  if (p1 < 1e-18) {                       // already diagonal
    int k = 0;
    if (C[4] < C[0]) k = 1;
    if (C[8] < C[k * 4]) k = 2;
    out[0] = out[1] = out[2] = 0.0f;
    out[k] = 1.0f;
    return true;
  }
  const double p2 = (C[0] - q) * (C[0] - q) + (C[4] - q) * (C[4] - q) +
                    (C[8] - q) * (C[8] - q) + 2.0 * p1;
  const double p = sqrt(p2 / 6.0);
  if (p < 1e-12) return false;
  double B[9];
  for (int i = 0; i < 9; i++) B[i] = (C[i] - ((i % 4 == 0) ? q : 0.0)) / p;
  double detB = B[0] * (B[4] * B[8] - B[5] * B[7]) -
                B[1] * (B[3] * B[8] - B[5] * B[6]) +
                B[2] * (B[3] * B[7] - B[4] * B[6]);
  double r = detB / 2.0;
  if (r > 1.0) r = 1.0;
  if (r < -1.0) r = -1.0;
  const double phi = acos(r) / 3.0;
  const double lambdaMin = q + 2.0 * p * cos(phi + 2.0 * M_PI / 3.0);

  double A[9];
  for (int i = 0; i < 9; i++) A[i] = C[i] - ((i % 4 == 0) ? lambdaMin : 0.0);
  const int pairs[3][2] = {{0, 1}, {0, 2}, {1, 2}};
  double best = -1.0;
  for (int k = 0; k < 3; k++) {
    const double *r1 = A + pairs[k][0] * 3, *r2 = A + pairs[k][1] * 3;
    const double c[3] = {r1[1] * r2[2] - r1[2] * r2[1],
                         r1[2] * r2[0] - r1[0] * r2[2],
                         r1[0] * r2[1] - r1[1] * r2[0]};
    const double n2 = c[0] * c[0] + c[1] * c[1] + c[2] * c[2];
    if (n2 > best) {
      best = n2;
      const double n1 = sqrt(n2);
      out[0] = (float)(c[0] / n1);
      out[1] = (float)(c[1] / n1);
      out[2] = (float)(c[2] / n1);
    }
  }
  return best > 1e-20;
}

bool ArmIMU::fitGeometry(const float (*g)[3], const float *angleDeg, int n,
                         const float axisHint[3], float axisOut[3], float levelOut[3]) {
  if (n < 6) return false;

  double mean[3] = {0, 0, 0};
  for (int i = 0; i < n; i++)
    for (int k = 0; k < 3; k++) mean[k] += g[i][k];
  for (int k = 0; k < 3; k++) mean[k] /= n;

  double C[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
  for (int i = 0; i < n; i++) {
    const double d[3] = {g[i][0] - mean[0], g[i][1] - mean[1], g[i][2] - mean[2]};
    for (int a = 0; a < 3; a++)
      for (int b = 0; b < 3; b++) C[a * 3 + b] += d[a] * d[b];
  }
  float axis[3];
  if (!smallestEigenvector(C, axis)) return false;
  if (!normalize(axis, axis)) return false;

  // The plane normal has no inherent sign; the motor's rotation sense does.
  float hint[3];
  if (normalize(axisHint, hint) && dot(axis, hint) < 0)
    for (int k = 0; k < 3; k++) axis[k] = -axis[k];

  // Every sample back-rotated to angle 0 should give the same vector, so
  // averaging them is the level reference with the noise beaten down.
  double acc[3] = {0, 0, 0};
  for (int i = 0; i < n; i++) {
    float back[3];
    rotateAbout(g[i], axis, angleDeg[i], back);
    for (int k = 0; k < 3; k++) acc[k] += back[k];
  }
  for (int k = 0; k < 3; k++) {
    axisOut[k] = axis[k];
    levelOut[k] = (float)(acc[k] / n);
  }
  return norm(levelOut) > 0.5f;
}

float ArmIMU::residualRmsDeg(const float (*g)[3], const float *angleDeg, int n,
                             const float axis[3], const float level[3]) {
  if (n < 1) return NAN;
  double ss = 0;
  int used = 0;
  for (int i = 0; i < n; i++) {
    // Wrap each prediction around its own sample so a station near the wrap
    // point does not produce a spurious 360 degree residual.
    float pred = angleFrom(g[i], axis, level, angleDeg[i]);
    if (isnan(pred)) continue;
    const double d = pred - angleDeg[i];
    ss += d * d;
    used++;
  }
  if (used < 1) return NAN;
  return (float)sqrt(ss / used);
}

float ArmIMU::armAngleDeg(float wrapCenterDeg) const {
  if (!filtSeeded_) return NAN;
  return armAngleDegFrom(gFilt_, wrapCenterDeg);
}
