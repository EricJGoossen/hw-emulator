#pragma once

// The checksums the wire formats use, written from their definitions:
//
//   Crc8Dji  -- CRC-8/MAXIM-DOW (reflected polynomial 0x8C), DJI serial's
//               header check, initial value 0xFF.
//   Crc16Dji -- CRC-16/MCRF4XX (reflected polynomial 0x8408), DJI serial's
//               frame check, initial value 0xFFFF.
//   Crc8Ld19 -- the LD19 lidar's packet check: MSB-first polynomial 0x4D,
//               initial value 0.

#include <cstdint>
#include <span>

namespace hwemu::protocol {

std::uint8_t Crc8Dji(std::span<const std::uint8_t> data, std::uint8_t init = 0xFF);
std::uint16_t Crc16Dji(std::span<const std::uint8_t> data, std::uint16_t init = 0xFFFF);
std::uint8_t Crc8Ld19(std::span<const std::uint8_t> data, std::uint8_t init = 0x00);

}  // namespace hwemu::protocol
