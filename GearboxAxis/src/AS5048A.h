// AS5048A 14-bit magnetic rotary encoder, SPI interface.
//
// Written against ams datasheet DS000298 v1-11 (c:\work\docs). The chip
// answers every SPI frame with the result of the PREVIOUS frame, so a read
// is two 16-bit transfers: the command, then a NOP that clocks the answer
// out. Frames are MSB first, SPI mode 1 (CPOL=0, CPHA=1), 10 MHz max; we
// run at 1 MHz because breadboard jumpers are not transmission lines.
//
// Command frame:  [15] even parity  [14] 1=read  [13:0] address
// Read frame:     [15] even parity  [14] error flag  [13:0] data
#pragma once

#include <Arduino.h>
#include <SPI.h>

class AS5048A {
 public:
  static constexpr uint16_t REG_NOP = 0x0000;
  static constexpr uint16_t REG_CLEAR_ERROR = 0x0001;
  static constexpr uint16_t REG_DIAG_AGC = 0x3FFD;
  static constexpr uint16_t REG_MAGNITUDE = 0x3FFE;
  static constexpr uint16_t REG_ANGLE = 0x3FFF;

  static constexpr float DEG_PER_LSB = 360.0f / 16384.0f;

  struct Diagnostics {
    uint8_t agc;        // 0..255 automatic gain; mid-range means a good air gap
    bool ocf;           // offset compensation finished (should be 1 after power-up)
    bool cof;           // CORDIC overflow - reading invalid
    bool compLow;       // magnetic field too STRONG (magnet too close)
    bool compHigh;      // magnetic field too WEAK (magnet too far)
    uint16_t magnitude; // CORDIC magnitude, 14 bit
  };

  // csPin: any output-capable GPIO. spi: a bus whose begin() you have
  // already called with the right pins.
  AS5048A(int csPin, SPIClass &spi = SPI, uint32_t clockHz = 1000000);

  // Sets up CS and performs a probe read. Returns true if the sensor
  // answered a valid, parity-correct frame with no error flag.
  bool begin();

  // 14-bit angle, 0..16383. False on SPI parity error or chip error flag.
  bool readAngleRaw(uint16_t &raw);

  // Angle in degrees 0..360. NAN on failure.
  float readAngleDeg();

  bool readDiagnostics(Diagnostics &d);

  // Reads and clears the chip's error register. Returns its bits:
  // bit0 framing error, bit1 command invalid, bit2 parity error.
  uint8_t clearErrorFlag();

  bool lastReadHadError() const { return lastError_; }
  uint32_t errorCount() const { return errorCount_; }

  // Bit 15 set so the 16-bit word has an even number of ones.
  static uint16_t withParity(uint16_t v);
  static bool parityOk(uint16_t v);

 private:
  uint16_t transfer(uint16_t frame);
  bool readRegister(uint16_t addr, uint16_t &data);

  int cs_;
  SPIClass &spi_;
  SPISettings settings_;
  bool lastError_ = false;
  uint32_t errorCount_ = 0;
};
