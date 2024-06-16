// Copyright (c) 2024 Quinton
// Licensed under the MIT License.

#ifndef DBUS_HPP_
#define DBUS_HPP_

#include <algorithm>
#include <cstdint>
#include <string>
#include <vector>

namespace dr16_dbus
{
  struct ReceiveDbus
  {
    int16_t ch0;
    int16_t ch1;
    int16_t ch2;
    int16_t ch3;
    uint8_t s0;
    uint8_t s1;
    int16_t wheel;

    int16_t x;
    int16_t y;
    int16_t z;

    uint8_t l;
    uint8_t r;
    uint16_t key;

  } __attribute__((packed));

  inline ReceiveDbus fromVector(const std::vector<uint8_t> &data)
  {
    ReceiveDbus dbus;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&dbus));
    return dbus;
  }

} // namespace dr16_dbus

#endif // DBUS_HPP_
