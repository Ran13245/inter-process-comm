// Copyright 2025 Chuangye Liu <chuangyeliu0206@gmail.com>
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include "../common.hpp"
#include "../compress_utils.hpp"
#include "../CRC.h"
#include "whole_body_msg.h"

struct WholeBodySender {
  using DataType                         = whole_body_msg;
  static constexpr uint8_t parser_type   = ParserType::Sender;
  static constexpr uint16_t header       = 0xFBFB;
  static constexpr size_t length         = 62;
  static constexpr std::string_view name = "whole_body_sender";

  static inline void Process(const DataType& in, std::span<std::byte>& out) {
    uint32_t tmp_32bits;
    memcpy(&out[0], &header, sizeof(uint16_t));
    memcpy(&out[2], &in.mask, sizeof(uint16_t));

    memcpy(&out[4], &in.cnt, sizeof(uint32_t));
    memcpy(&out[8], &in.time, sizeof(uint64_t));
    // base pos
    memcpy(&out[16], in.base_pos.data(), sizeof(float) * 3);
    // base quaternion
    tmp_32bits = EncodeQuaternion<float>(in.base_quat.data());
    memcpy(&out[28], &tmp_32bits, sizeof(uint32_t));
    // base linear velocity
    tmp_32bits = Encode3D<float, 10>(in.base_lin_vel[0], in.base_lin_vel[1], in.base_lin_vel[2], 3.0);
    memcpy(&out[32], &tmp_32bits, sizeof(uint32_t));
    // base angular velocity
    tmp_32bits = Encode3D<float, 10>(in.base_ang_vel[0], in.base_ang_vel[1], in.base_ang_vel[2], 3.14);
    memcpy(&out[36], &tmp_32bits, sizeof(uint32_t));
    // left arm joint position
    tmp_32bits = Encode3D<float, 10>(in.left_arm_joint_pos[0], in.left_arm_joint_pos[1], in.left_arm_joint_pos[2], 3.1415926);
    memcpy(&out[40], &tmp_32bits, sizeof(uint32_t));
    tmp_32bits = Encode3D<float, 10>(in.left_arm_joint_pos[3], in.left_arm_joint_pos[4], in.left_arm_joint_pos[5], 3.1415926);
    memcpy(&out[44], &tmp_32bits, sizeof(uint32_t));
    // right arm joint position
    tmp_32bits = Encode3D<float, 10>(in.right_arm_joint_pos[0], in.right_arm_joint_pos[1], in.right_arm_joint_pos[2], 3.1415926);
    memcpy(&out[48], &tmp_32bits, sizeof(uint32_t));
    tmp_32bits = Encode3D<float, 10>(in.right_arm_joint_pos[3], in.right_arm_joint_pos[4], in.right_arm_joint_pos[5], 3.1415926);
    memcpy(&out[52], &tmp_32bits, sizeof(uint32_t));
    // grip force
    tmp_32bits = Encode2D<float, 16>(in.left_grip, in.right_grip);
    memcpy(&out[56], &tmp_32bits, sizeof(uint32_t));
    // CRC
    uint16_t crc = CRC::CalculateBits(out.data(), 60, CRC::CRC_16_KERMIT());
    memcpy(&out[60], &crc, sizeof(uint16_t));
    // total 62 bytes
    return;
  }
};