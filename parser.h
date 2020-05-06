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

#ifndef LIDAR_OUSTER_SRC_PARSE_H
#define LIDAR_OUSTER_SRC_PARSE_H

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "cyber/base/concurrent_object_pool.h"
#include "cyber/cyber.h"
#include "modules/drivers/ouster/const_var.h"
#include "modules/drivers/ouster/proto/config.pb.h"
#include "modules/drivers/ouster/proto/ouster.pb.h"
#include "modules/drivers/ouster/type_defs.h"
#include "modules/drivers/proto/pointcloud.pb.h"
#include "modules/drivers/ouster/os1.h"
#include "modules/drivers/ouster/lidar_scan.h"

namespace apollo {
namespace drivers {
namespace ouster {

using apollo::drivers::PointCloud;
using apollo::drivers::PointXYZIT;
using apollo::drivers::ouster::OusterScan;

using apollo::cyber::Node;
using apollo::cyber::Writer;
using apollo::cyber::base::CCObjectPool;

class Parser {
 public:
  Parser(const std::shared_ptr<Node>& node, const Config& conf);
  virtual ~Parser();
  void Parse(const uint8_t* data, int size, 
                  const OS1::packet_format& pf, bool* is_end);
  void ParseIMU(const uint8_t* buf, const OS1::packet_format& pf);
  bool Parse(const std::shared_ptr<OusterScan>& scan);
  bool Init();
  void get_sensor_params(int &lidar_socker, int &imu_socker, 
                      int &socket_number, OS1::sensor_info& info);

 private:
  std::thread online_calibration_thread_;
  bool CheckIsEnd(bool is_end);
  void LoadSenseInfo(OS1::sensor_info& info);
  void PublishRawPointCloud(int ret = -1);

  bool inited_ = false;
  void Stop();
  std::atomic<bool> running_ = {true};

 protected:
  virtual void init_xyz_lut() = 0;
  virtual void ParseLidarPacket(const uint8_t* buf, const int len, 
                            const OS1::packet_format& pf, bool* is_end) = 0;  
  virtual void ParseImuPacket(const uint8_t* buf, const OS1::packet_format& pf) = 0;
  void ResetRawPointCloud();

  std::shared_ptr<Node> node_;
  Config conf_;
  std::shared_ptr<Writer<PointCloud>> raw_pointcloud_writer_;
  int pool_size_ = 8;
  int pool_index_ = 0;
  int seq_index_ = 0;
  std::deque<std::shared_ptr<PointCloud>> raw_pointcloud_pool_;
  std::shared_ptr<PointCloud> raw_pointcloud_out_ = nullptr;

  int tz_second_ = 0;
  uint32_t min_packets_ = OUSTER128_MIN_PACKETS;
  uint32_t max_packets_ = OUSTER128_MAX_PACKETS;
  uint32_t packet_nums_ = 0;

  int32_t cur_f_id_;
  int64_t scan_ts_;
  uint16_t first_m_id_;
  uint16_t last_m_id_;
  int pointcloud_w_;
  int pointcloud_h_;
  OS1::sensor_info info_{};
  int lidar_socker_ = -1;
  int imu_socker_ = -1;
  int socket_number_ = -1;

};

/***********************OUSTER128***********************/
class Ouster128Parser : public Parser {
 public:
  Ouster128Parser(const std::shared_ptr<Node>& node, const Config& conf);
  ~Ouster128Parser();

 protected:
  void init_xyz_lut() override;
  void ParseLidarPacket(const uint8_t* buf, const int len,
                     const OS1::packet_format& pf, bool* is_end) override;  
  virtual void ParseImuPacket(const uint8_t* buf, const OS1::packet_format& pf) override;

 private:
  OS1::xyz_lut  xyz_lut_; 
  int64_t n_imu_packets_ = 0;
};

class ParserFactory {
 public:
  static Parser* CreateParser(const std::shared_ptr<Node>& node,
                              const Config& conf);
};

}  // namespace ouster
}  // namespace drivers
}  // namespace apollo

#endif
