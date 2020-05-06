/******************************************************************************
 * Copyright 2020 The Whale Dynamics Authors. All Rights Reserved.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *****************************************************************************/

#ifndef LIDAR_OUSTER_DRIVER_H_
#define LIDAR_OUSTER_DRIVER_H_

#include <atomic>
#include <condition_variable>
#include <deque>
#include <functional>
#include <list>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "cyber/cyber.h"
#include "modules/drivers/ouster/const_var.h"
#include "modules/drivers/ouster/driver.h"
#include "modules/drivers/ouster/parser.h"
#include "modules/drivers/ouster/proto/config.pb.h"
#include "modules/drivers/ouster/proto/ouster.pb.h"
#include "modules/drivers/ouster/type_defs.h"
#include "modules/drivers/ouster/os1.h"

namespace apollo {
namespace drivers {
namespace ouster {

using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::drivers::ouster::OusterScan;

class OusterDriver {
 public:
  OusterDriver(const std::shared_ptr<Node>& node, const Config& conf,
              const std::shared_ptr<Parser>& parser)
      : node_(node), conf_(conf), parser_(parser) {}
  ~OusterDriver() { Stop(); }
  bool Init();

 private:
  std::shared_ptr<Node> node_ = nullptr;
  Config conf_;
  std::shared_ptr<Parser> parser_ = nullptr;
  std::shared_ptr<Writer<OusterScan>> scan_writer_ = nullptr;
  std::mutex packet_mutex_;
  std::condition_variable packet_condition_;
  std::thread poll_thread_;
  std::thread process_thread_;
  std::atomic<bool> running_ = {true};
  std::deque<std::shared_ptr<OusterScan>> scan_buffer_;
  int scan_buffer_size_ = 10;
  int index_ = 0;
  int tz_second_ = 0;
  std::vector<std::shared_ptr<OusterPacket>> pkt_buffer_;
  int pkt_index_ = 0;
  int pkt_buffer_capacity_ = 0;
  std::list<std::shared_ptr<OusterPacket>> pkt_queue_;
  int socket_lidar_ = -1;
  int socket_imu_ = -1;
  int socket_count_ = -1;
  size_t imu_packet_size_ = 0;
  size_t lidar_packet_size_ = 0;
  OS1::sensor_info sensor_info_{};

 private:
  void PollThread();
  void ProcessThread();
  void ProcessIMU(const OusterPacket& pkt, const OS1::packet_format& pf);
  OS1::client_state poll_client(int lidar_fd, int imu_fd, int timeout_sec = 1);
  void receive_data(int fd, size_t recv_max_len, int max_cnt = 10);

  void Stop() {
    AINFO << "driver stoping...";
    running_.store(false);
    packet_condition_.notify_all();
    if (poll_thread_.joinable()) {
      poll_thread_.join();
    }
    if (process_thread_.joinable()) {
      process_thread_.join();
    }
  }
};

}  // namespace ouster
}  // namespace drivers
}  // namespace apollo

#endif
