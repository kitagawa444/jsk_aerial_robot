#ifndef SPINAL_DSHOT_PROTOCOL_H
#define SPINAL_DSHOT_PROTOCOL_H
#include <stdint.h>
#include <stddef.h>
#include <math.h>

namespace dshot_protocol
{
// Reject non-finite inputs and clamp finite throttle requests to full scale.
inline uint16_t throttle(float duty)
{
  if (!isfinite(duty) || duty <= 0.5f) return 0;
  if (duty > 1.0f) duty = 1.0f;
  return static_cast<uint16_t>((duty - 0.5f) * 2.0f * 1999.0f + 48.0f);
}
inline uint16_t packet(uint16_t value, bool telemetry)
{
  const uint16_t payload = (value << 1) | (telemetry ? 1 : 0);
  return (payload << 4) | ((payload ^ (payload >> 4) ^ (payload >> 8)) & 0xf);
}
struct Timing { uint32_t period, zero_high, one_high; };
inline Timing timing(uint32_t clock, uint32_t bitrate)
{
  const uint32_t ticks = (clock + bitrate / 2) / bitrate;
  return {ticks - 1, (ticks * 3 + 4) / 8, (ticks * 3 + 2) / 4};
}
inline uint8_t crc8(const uint8_t* bytes, size_t size)
{
  uint8_t crc = 0;
  for (size_t i = 0; i < size; ++i)
  {
    crc ^= bytes[i];
    for (unsigned bit = 0; bit < 8; ++bit)
      crc = (crc & 0x80) ? (crc << 1) ^ 7 : crc << 1;
  }
  return crc;
}
class KissParser
{
public:
  void reset() { size_ = 0; }
  bool push(uint8_t byte, uint8_t* frame)
  {
    bytes_[size_++] = byte;
    if (size_ < 10) return false;
    if (crc8(bytes_, 9) == bytes_[9])
    {
      for (unsigned i = 0; i < 10; ++i) frame[i] = bytes_[i];
      size_ = 0;
      return true;
    }
    // KISS has no sync byte; slide one byte to recover alignment.
    for (unsigned i = 1; i < 10; ++i) bytes_[i - 1] = bytes_[i];
    size_ = 9;
    return false;
  }
private:
  uint8_t bytes_[10] = {};
  unsigned size_ = 0;
};
}
#endif
