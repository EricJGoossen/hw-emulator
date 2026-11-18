#include "hwemu/protocol/spi_imu.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <numbers>

#include "hwemu/protocol/bytes.hpp"

namespace hwemu::protocol::spi_imu {
namespace {

constexpr double kRadToDeg = 180.0 / std::numbers::pi;

std::int16_t Sat16(double v) {
  return static_cast<std::int16_t>(std::lround(std::clamp(v, -32768.0, 32767.0)));
}

}  // namespace

RegisterFile::RegisterFile() = default;

std::vector<std::uint8_t> RegisterFile::Transfer(std::span<const std::uint8_t> tx) {
  std::lock_guard<std::mutex> lock(mutex_);
  std::vector<std::uint8_t> rx(tx.size(), 0);
  if (tx.empty()) {
    return rx;
  }
  const bool read = (tx[0] & 0x80U) != 0;
  auto reg = static_cast<std::uint8_t>(tx[0] & 0x7FU);
  if (read) {
    const std::size_t dummy = ReadDummyBytes();
    for (std::size_t i = 1; i < tx.size(); ++i) {
      if (i <= dummy) {
        rx[i] = 0xFF;
        continue;
      }
      rx[i] = regs_[reg & 0x7FU];
      if (AutoIncrement()) {
        reg = static_cast<std::uint8_t>((reg + 1U) & 0x7FU);
      }
    }
  } else {
    for (std::size_t i = 1; i < tx.size(); ++i) {
      Write(reg, tx[i]);
      if (AutoIncrement()) {
        reg = static_cast<std::uint8_t>((reg + 1U) & 0x7FU);
      }
    }
  }
  return rx;
}

void RegisterFile::SetSample(const Sample& sample) {
  std::lock_guard<std::mutex> lock(mutex_);
  last_ = sample;
  Encode(sample);
}

std::uint8_t RegisterFile::Peek(std::uint8_t reg) const {
  std::lock_guard<std::mutex> lock(mutex_);
  return regs_[reg & 0x7FU];
}

// ---- MPU6500 --------------------------------------------------------------
void Mpu6500::Reset() {
  regs_.fill(0);
  regs_[0x6B] = 0x01;  // PWR_MGMT_1
  regs_[0x75] = 0x70;  // WHO_AM_I
}

double Mpu6500::GyroLsbPerDps() const {
  static constexpr std::array<double, 4> kScale = {131.0, 65.5, 32.8, 16.4};
  return kScale[(regs_[0x1B] >> 3U) & 0x3U];
}

double Mpu6500::AccelLsbPerG() const {
  static constexpr std::array<double, 4> kScale = {16384.0, 8192.0, 4096.0, 2048.0};
  return kScale[(regs_[0x1C] >> 3U) & 0x3U];
}

void Mpu6500::Encode(const Sample& s) {
  const double g = AccelLsbPerG() / kGravity;
  const double w = GyroLsbPerDps() * kRadToDeg;
  for (std::size_t i = 0; i < 3; ++i) {
    PutI16Be(&regs_[0x3B + (2 * i)], Sat16(s.accel[i] * g));
    PutI16Be(&regs_[0x43 + (2 * i)], Sat16(s.gyro[i] * w));
  }
  PutI16Be(&regs_[0x41], Sat16((s.temperature - 21.0) * 333.87));
}

void Mpu6500::Write(std::uint8_t reg, std::uint8_t value) {
  if (reg == 0x75) {
    return;  // read-only
  }
  if (reg == 0x6B && (value & 0x80U) != 0) {
    Reset();  // DEVICE_RESET self-clears
    Encode(last_);
    return;
  }
  regs_[reg] = value;
  if (reg == 0x1B || reg == 0x1C) {
    Encode(last_);
  }
}

// ---- ISM330DHCX -----------------------------------------------------------
void Ism330dhcx::Reset() {
  regs_.fill(0);
  regs_[0x0F] = 0x6B;  // WHO_AM_I
  regs_[0x12] = 0x04;  // CTRL3_C: IF_INC
}

double Ism330dhcx::GyroMdpsPerLsb() const {
  const std::uint8_t ctrl2 = regs_[0x11];
  if ((ctrl2 & 0x01U) != 0) {
    return 140.0;  // FS_4000
  }
  if ((ctrl2 & 0x02U) != 0) {
    return 4.375;  // FS_125
  }
  static constexpr std::array<double, 4> kScale = {8.75, 17.5, 35.0, 70.0};
  return kScale[(ctrl2 >> 2U) & 0x3U];
}

double Ism330dhcx::AccelMgPerLsb() const {
  // FS_XL: 00 2g, 01 16g, 10 4g, 11 8g.
  static constexpr std::array<double, 4> kScale = {0.061, 0.488, 0.122, 0.244};
  return kScale[(regs_[0x10] >> 2U) & 0x3U];
}

void Ism330dhcx::Encode(const Sample& s) {
  const double g = 1000.0 / (AccelMgPerLsb() * kGravity);
  const double w = 1000.0 * kRadToDeg / GyroMdpsPerLsb();
  for (std::size_t i = 0; i < 3; ++i) {
    PutI16Le(&regs_[0x22 + (2 * i)], Sat16(s.gyro[i] * w));
    PutI16Le(&regs_[0x28 + (2 * i)], Sat16(s.accel[i] * g));
  }
  PutI16Le(&regs_[0x20], Sat16((s.temperature - 25.0) * 256.0));
}

void Ism330dhcx::Write(std::uint8_t reg, std::uint8_t value) {
  if (reg == 0x0F) {
    return;
  }
  if (reg == 0x12 && (value & 0x01U) != 0) {
    Reset();  // SW_RESET self-clears
    Encode(last_);
    return;
  }
  regs_[reg] = value;
  if (reg == 0x10 || reg == 0x11) {
    Encode(last_);
  }
}

// ---- BMI088 accelerometer -------------------------------------------------
void Bmi088Accel::Reset() {
  regs_.fill(0);
  regs_[0x00] = 0x1E;        // ACC_CHIP_ID
  regs_[0x40] = 0xA8;        // ACC_CONF
  regs_[0x41 & 0x7F] = 0x01;  // ACC_RANGE: 6 g
}

double Bmi088Accel::RangeG() const { return 3.0 * static_cast<double>(1U << (regs_[0x41] & 0x3U)); }

void Bmi088Accel::Encode(const Sample& s) {
  const double lsb_per_ms2 = 32768.0 / (RangeG() * kGravity);
  for (std::size_t i = 0; i < 3; ++i) {
    PutI16Le(&regs_[0x12 + (2 * i)], Sat16(s.accel[i] * lsb_per_ms2));
  }
  // TEMP_MSB 0x22, TEMP_LSB 0x23 bits 7:5: an 11-bit two's complement value.
  const long raw = std::lround(std::clamp((s.temperature - 23.0) / 0.125, -1024.0, 1023.0));
  const auto u = static_cast<std::uint16_t>(raw & 0x7FF);
  regs_[0x22] = static_cast<std::uint8_t>(u >> 3U);
  regs_[0x23] = static_cast<std::uint8_t>((u & 0x7U) << 5U);
}

void Bmi088Accel::Write(std::uint8_t reg, std::uint8_t value) {
  if (reg == 0x00) {
    return;
  }
  if (reg == 0x7E && value == 0xB6) {
    Reset();  // ACC_SOFTRESET
    Encode(last_);
    return;
  }
  regs_[reg] = value;
  if (reg == 0x41) {
    Encode(last_);
  }
}

// ---- BMI088 gyroscope -----------------------------------------------------
void Bmi088Gyro::Reset() {
  regs_.fill(0);
  regs_[0x00] = 0x0F;  // GYRO_CHIP_ID
  regs_[0x0F] = 0x00;  // GYRO_RANGE: 2000 dps
}

double Bmi088Gyro::LsbPerDps() const {
  static constexpr std::array<double, 5> kScale = {16.384, 32.768, 65.536, 131.072, 262.144};
  const std::uint8_t range = regs_[0x0F] & 0x7U;
  return kScale[std::min<std::size_t>(range, 4)];
}

void Bmi088Gyro::Encode(const Sample& s) {
  const double w = LsbPerDps() * kRadToDeg;
  for (std::size_t i = 0; i < 3; ++i) {
    PutI16Le(&regs_[0x02 + (2 * i)], Sat16(s.gyro[i] * w));
  }
}

void Bmi088Gyro::Write(std::uint8_t reg, std::uint8_t value) {
  if (reg == 0x00) {
    return;
  }
  if (reg == 0x14 && value == 0xB6) {
    Reset();  // GYRO_SOFTRESET
    Encode(last_);
    return;
  }
  regs_[reg] = value;
  if (reg == 0x0F) {
    Encode(last_);
  }
}

std::unique_ptr<RegisterFile> Make(const std::string& chip) {
  if (chip == "mpu6500") return std::make_unique<Mpu6500>();
  if (chip == "ism330dhcx" || chip == "ism330") return std::make_unique<Ism330dhcx>();
  if (chip == "bmi088_accel") return std::make_unique<Bmi088Accel>();
  if (chip == "bmi088_gyro") return std::make_unique<Bmi088Gyro>();
  return nullptr;
}

}  // namespace hwemu::protocol::spi_imu
