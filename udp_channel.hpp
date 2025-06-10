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

#include <span>
#include <unordered_map>
#include <utility>
#include <asio.hpp>

#include "msgs/common.hpp"
#include "itc/MsgQueue.hpp"

template <typename T>
concept ValidParser = true;

template <ValidParser... Parsers>
class UdpChannel {
public:
  UdpChannel() = delete;

  UdpChannel(asio::io_context &io_context, std::string local_ip, int local_port,
             std::string remote_ip, int remote_port)
      : local_endpoint_(asio::ip::make_address(local_ip), local_port),
        remote_endpoint_(asio::ip::make_address(remote_ip), remote_port),
        socket_(io_context, asio::ip::udp::v4()),
        timer_(io_context) {
    socket_.bind(local_endpoint_);
  }

  // forcefully bind a constexpr name
  template <std::size_t N>
  void bind_message_queue(const char (&name)[N], const ParserType attribute, MsgQueue &mq) {
    std::string_view sv{name, N - 1};
    MsgQueue *mq_ptr = &mq;
    if (attribute == ParserType::Sender) {
      send_mq_vec_.emplace_back(std::make_pair(sv, mq_ptr));
    } else if (attribute == ParserType::Receiver) {
      recv_mq_map_.emplace(sv, mq_ptr);
    }
    return;
  }

  bool enable_receiver() {
    // do bind checking
    bool mq_registered = (((Parsers::parser_type == ParserType::Receiver)
                           && (recv_mq_map_.find(Parsers::name) != recv_mq_map_.end()))
                          || ...);
    if (!mq_registered) return false;

    asio::ip::udp::endpoint tmp_endpoint = remote_endpoint_;
    this->socket_.async_receive_from(asio::buffer(recv_buffer_), remote_endpoint_,
                                     std::bind(&UdpChannel::receiver_handler, this,
                                               std::placeholders::_1, std::placeholders::_2));
    return true;
  }

  bool enable_sender() {
    bool mq_registered = (((Parsers::parser_type == ParserType::Sender)
                           && (std::ranges::find_if(send_mq_vec_,
                                                    [](const auto &curr_mq) {
                                                      return curr_mq.first == Parsers::name;
                                                    })
                               != send_mq_vec_.end()))
                          || ...);
    if (!mq_registered) return false;
    timer_.async_wait(std::bind(&UdpChannel::timer_handler, this, std::placeholders::_1));
    return true;
  }

private:
  void timer_handler(const asio::error_code &ec) {
    if (!ec) {
      auto curr_mq = send_mq_vec_[loop_cnt];
      if (!curr_mq.second->empty()) {
        std::span<std::byte> buffer_view(this->send_buffer_);
        bool matched
            = (([this, &curr_mq, &buffer_view]() -> bool {
                 if constexpr (Parsers::parser_type == ParserType::Sender) {
                   if (curr_mq.first == Parsers::name) {
                     typename Parsers::DataType data;
                     curr_mq.second->dequeue(data);
                     Parsers::Process(data, buffer_view);
                     this->socket_.async_send_to(
                         asio::buffer(this->send_buffer_), this->remote_endpoint_,
                         [](const asio::error_code &error, std::size_t bytes_transferred) {});
                     return true;
                   } else
                     return false;
                 }
                 return false;
               }())
               || ...);
      }
      loop_cnt++;
      // reset counter
      if (loop_cnt >= send_mq_vec_.size()) loop_cnt = 0;
    }
    timer_.expires_after(asio::chrono::milliseconds(1));
    timer_.async_wait(std::bind(&UdpChannel::timer_handler, this, std::placeholders::_1));
    return;
  }

  void receiver_handler(const asio::error_code &ec, std::size_t bytes_transferred) {
    if (!ec && bytes_transferred > 0) {
      std::span<std::byte> buffer_view(this->recv_buffer_);

      uint16_t header = *reinterpret_cast<uint16_t *>(buffer_view.data());

      bool matched = ([this, header, &buffer_view]() -> bool {
        typename Parsers::DataType recv_data;
        if constexpr (Parsers::parser_type == ParserType::Receiver) {
          if (header == Parsers::header) {
            Parsers::Process(buffer_view, recv_data);
          } else {
            return false;
          }
        } else {
          return false;
        }

        auto it = this->recv_mq_map_.find(Parsers::name);
        if (it != this->recv_mq_map_.end()) {
          it->second->enqueue(recv_data);
        }
        return true;
      }() || ...);

      // do alert here
    }
    asio::ip::udp::endpoint tmp_endpoint = remote_endpoint_;
    this->socket_.async_receive_from(asio::buffer(recv_buffer_), tmp_endpoint,
                                     std::bind(&UdpChannel::receiver_handler, this,
                                               std::placeholders::_1, std::placeholders::_2));
  }

  asio::steady_timer timer_;
  asio::ip::udp::socket socket_;
  asio::ip::udp::endpoint local_endpoint_;
  asio::ip::udp::endpoint remote_endpoint_;

  std::vector<std::pair<std::string_view, MsgQueue *>> send_mq_vec_;
  std::unordered_map<std::string_view, MsgQueue *> recv_mq_map_;
  std::array<std::byte, 1500> send_buffer_;
  std::array<std::byte, 1500> recv_buffer_;

  uint32_t loop_cnt = 0;
};