// Copyright (c) 2022 ChenJun
// Licensed under the Apache-2.0 License.

#ifndef RM_VISION_CONTROLLER__PACKET_HPP_
#define RM_VISION_CONTROLLER__PACKET_HPP_

#include <algorithm>
#include <cstdint>
#include <vector>

namespace rm_vision_controller
{
  struct ReceivePacket
  {
    uint8_t detect_color;  // 0-red 1-blue
    bool reset_tracker;
    
    float roll;
    float pitch;
    float yaw;
    float aim_x;
    float aim_y;
    float aim_z;

    uint16_t sentry_hp;
    uint16_t base_hp;
    uint16_t ammo;
    uint16_t remaining_time;
  } __attribute__((packed));

  inline ReceivePacket fromVector(const std::vector<uint8_t> & data)
  {
    ReceivePacket packet;
    std::copy(data.begin(), data.end(), reinterpret_cast<uint8_t *>(&packet));
    return packet;
  }

  template <typename T>
  inline std::vector<uint8_t> toVector(const T & packet)
  {
    std::vector<uint8_t> data(sizeof(T));
    std::copy(
      reinterpret_cast<const uint8_t *>(&packet), reinterpret_cast<const uint8_t *>(&packet) + sizeof(T),
      data.begin());
    return data;
  }

}  // namespace rm_vision_controller

#endif  // RM_VISION_CONTROLLER__PACKET_HPP_