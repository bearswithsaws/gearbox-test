#include "AS5048A.h"

AS5048A::AS5048A(int csPin, SPIClass &spi, uint32_t clockHz)
    : cs_(csPin), spi_(spi), settings_(clockHz, MSBFIRST, SPI_MODE1) {}

uint16_t AS5048A::withParity(uint16_t v) {
  v &= 0x7FFF;
  uint16_t x = v;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  if (x & 1) v |= 0x8000;
  return v;
}

bool AS5048A::parityOk(uint16_t v) {
  uint16_t x = v;
  x ^= x >> 8;
  x ^= x >> 4;
  x ^= x >> 2;
  x ^= x >> 1;
  return (x & 1) == 0;
}

bool AS5048A::begin() {
  pinMode(cs_, OUTPUT);
  digitalWrite(cs_, HIGH);
  delay(1);
  // First frame after power-up can carry a stale error flag; clear it,
  // then check we get a sane angle.
  clearErrorFlag();
  uint16_t raw;
  if (!readAngleRaw(raw)) return false;
  // A floating MISO reads all zeros, which has even parity and no error
  // flag, so a parity-correct angle alone does not prove a chip is there.
  // A live chip reports offset-compensation-finished and a nonzero
  // CORDIC magnitude; a missing one reports neither.
  Diagnostics d;
  if (!readDiagnostics(d)) return false;
  return d.ocf && d.magnitude > 0;
}

uint16_t AS5048A::transfer(uint16_t frame) {
  spi_.beginTransaction(settings_);
  digitalWrite(cs_, LOW);
  uint16_t r = spi_.transfer16(frame);
  digitalWrite(cs_, HIGH);
  spi_.endTransaction();
  // Datasheet: CS must idle high for at least 350 ns between frames.
  delayMicroseconds(1);
  return r;
}

bool AS5048A::readRegister(uint16_t addr, uint16_t &data) {
  transfer(withParity(0x4000 | (addr & 0x3FFF)));   // bit14 = read
  uint16_t resp = transfer(withParity(REG_NOP));    // clocks the answer out
  bool ok = parityOk(resp) && !(resp & 0x4000);
  lastError_ = !ok;
  if (!ok) {
    errorCount_++;
    if (resp & 0x4000) clearErrorFlag();  // otherwise the flag latches
  }
  data = resp & 0x3FFF;
  return ok;
}

bool AS5048A::readAngleRaw(uint16_t &raw) {
  return readRegister(REG_ANGLE, raw);
}

float AS5048A::readAngleDeg() {
  uint16_t raw;
  if (!readAngleRaw(raw)) return NAN;
  return raw * DEG_PER_LSB;
}

bool AS5048A::readDiagnostics(Diagnostics &d) {
  uint16_t diag, mag;
  if (!readRegister(REG_DIAG_AGC, diag)) return false;
  if (!readRegister(REG_MAGNITUDE, mag)) return false;
  d.agc = diag & 0xFF;
  d.ocf = diag & (1 << 8);
  d.cof = diag & (1 << 9);
  d.compLow = diag & (1 << 10);
  d.compHigh = diag & (1 << 11);
  d.magnitude = mag;
  return true;
}

uint8_t AS5048A::clearErrorFlag() {
  transfer(withParity(0x4000 | REG_CLEAR_ERROR));
  uint16_t resp = transfer(withParity(REG_NOP));
  return resp & 0x07;
}
