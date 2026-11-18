#pragma once

// SPI register maps of the IMUs on the main boards and turret boards, as
// register files the firmware reads over pinbus SPI (transport/pinbus.hpp).
// Each takes a whole chip-select transaction -- an address byte (bit 7 set
// for a read) then data -- and answers it byte for byte as the chip would,
// with the data registers filled from SetSample() in the chip's own units,
// scaled by its own full-scale configuration registers.
//
//   Mpu6500     WHO_AM_I 0x75 = 0x70; ACCEL_XOUT_H 0x3B: accel xyz, temp,
//               gyro xyz, big-endian; FS from GYRO_CONFIG 0x1B / ACCEL_CONFIG
//               0x1C; temp = raw / 333.87 + 21.
//   Ism330dhcx  WHO_AM_I 0x0F = 0x6B; OUT_TEMP_L 0x20, OUTX_L_G 0x22,
//               OUTX_L_A 0x28, little-endian; FS from CTRL2_G 0x11 /
//               CTRL1_XL 0x10; temp = raw / 256 + 25; IF_INC in CTRL3_C.
//   Bmi088Accel CHIP_ID 0x00 = 0x1E; ACC_X_LSB 0x12; ACC_RANGE 0x41; TEMP
//               0x22/0x23 (11 bit, 0.125 degC, +23); a read clocks one dummy
//               byte before the data, as the chip's SPI does.
//   Bmi088Gyro  CHIP_ID 0x00 = 0x0F; RATE_X_LSB 0x02; GYRO_RANGE 0x0F.
//
// All auto-increment across a burst read.

#include <array>
#include <cstdint>
#include <memory>
#include <mutex>
#include <span>
#include <string>
#include <vector>

namespace hwemu::protocol::spi_imu {

// One IMU sample in SI units: rad/s, m/s^2 (specific force), degC.
struct Sample {
  std::array<double, 3> gyro{};
  std::array<double, 3> accel{};
  double temperature = 25.0;
};

inline constexpr double kGravity = 9.80665;

class RegisterFile {
 public:
  virtual ~RegisterFile() = default;
  // One chip-select transaction: returns as many bytes as `tx` holds.
  // Thread-safe against SetSample.
  std::vector<std::uint8_t> Transfer(std::span<const std::uint8_t> tx);
  void SetSample(const Sample& sample);
  std::uint8_t Peek(std::uint8_t reg) const;
  virtual std::string Chip() const = 0;

 protected:
  RegisterFile();
  // Resets every register to its power-on value.
  virtual void Reset() = 0;
  // Writes `sample` into the data registers, at the configured ranges.
  virtual void Encode(const Sample& sample) = 0;
  // A write of `value` to `reg`: stores it, and runs any side effect.
  virtual void Write(std::uint8_t reg, std::uint8_t value) { regs_[reg] = value; }
  virtual bool AutoIncrement() const { return true; }
  // Bytes clocked out between the address and the first data byte.
  virtual std::size_t ReadDummyBytes() const { return 0; }

  std::array<std::uint8_t, 128> regs_{};
  Sample last_;

 private:
  mutable std::mutex mutex_;
};

class Mpu6500 final : public RegisterFile {
 public:
  Mpu6500() { Reset(); }
  std::string Chip() const override { return "mpu6500"; }
  double GyroLsbPerDps() const;
  double AccelLsbPerG() const;

 protected:
  void Reset() override;
  void Encode(const Sample& s) override;
  void Write(std::uint8_t reg, std::uint8_t value) override;
};

class Ism330dhcx final : public RegisterFile {
 public:
  Ism330dhcx() { Reset(); }
  std::string Chip() const override { return "ism330dhcx"; }
  double GyroMdpsPerLsb() const;
  double AccelMgPerLsb() const;

 protected:
  void Reset() override;
  void Encode(const Sample& s) override;
  void Write(std::uint8_t reg, std::uint8_t value) override;
  bool AutoIncrement() const override { return (regs_[0x12] & 0x04U) != 0; }
};

class Bmi088Accel final : public RegisterFile {
 public:
  Bmi088Accel() { Reset(); }
  std::string Chip() const override { return "bmi088_accel"; }
  double RangeG() const;

 protected:
  void Reset() override;
  void Encode(const Sample& s) override;
  void Write(std::uint8_t reg, std::uint8_t value) override;
  std::size_t ReadDummyBytes() const override { return 1; }
};

class Bmi088Gyro final : public RegisterFile {
 public:
  Bmi088Gyro() { Reset(); }
  std::string Chip() const override { return "bmi088_gyro"; }
  double LsbPerDps() const;

 protected:
  void Reset() override;
  void Encode(const Sample& s) override;
  void Write(std::uint8_t reg, std::uint8_t value) override;
};

// "mpu6500", "ism330dhcx", "bmi088_accel", "bmi088_gyro"; null otherwise.
std::unique_ptr<RegisterFile> Make(const std::string& chip);

}  // namespace hwemu::protocol::spi_imu
