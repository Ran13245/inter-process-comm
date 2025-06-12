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
#include "pcd_msg.h"

struct PcdSender {
  using DataType                         = dummy_msg;
  static constexpr uint8_t parser_type   = ParserType::DirectSender;
  static constexpr uint16_t header       = 0xD0FD;
  static constexpr size_t length         = 1356; 
  //warning!!! length = PointPacket<CompressedPoint, PACKET_MTU>::TotalByte,	ralated to PACKET_MTU = 1400
  static constexpr std::string_view name = "pcd_sender";

  static constexpr void Process(const DataType& in, std::span<std::byte>& out) {//! unused
//     std::memcpy(out.data(), &header, 2);
//     std::memcpy(out.data() + 2, &in, 4);
    return;
  }
};