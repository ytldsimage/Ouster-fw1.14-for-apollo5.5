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

#include <cmath>
#include <memory>
#include <string>

#include "modules/drivers/ouster/parser.h"

namespace apollo {
namespace drivers {
namespace ouster {

Parser::Parser(const std::shared_ptr<Node>& node, const Config& conf)
    : node_(node), conf_(conf) {
  tz_second_ = conf_.time_zone() * 3600;
}

Parser::~Parser() { Stop(); }

bool Parser::Init() {
  if (inited_) {
    return true;
  }

  LoadSenseInfo(info_);

  cur_f_id_ = -1;
  scan_ts_ = -1;
  first_m_id_ = 0;
  last_m_id_ = 0;  

  // init writer
  raw_pointcloud_writer_ =
      node_->CreateWriter<PointCloud>(conf_.pointcloud_channel());

  if (raw_pointcloud_writer_ == nullptr) {
    AERROR << "create writer:" << conf_.pointcloud_channel()
           << " error, check cyber is init?";
    return false;
  }
  else
  {
    AINFO << "create writer:" << conf_.pointcloud_channel()
           << " successfully";
  }

  raw_pointcloud_pool_.resize(pool_size_);
  for (int i = 0; i < pool_size_; i++) {
    raw_pointcloud_pool_[i] = std::make_shared<PointCloud>();
    if (raw_pointcloud_pool_[i] == nullptr) {
      AERROR << "make shared PointCloud error,oom";
      return false;
    }
    raw_pointcloud_pool_[i]->mutable_point()->Reserve(70000);
  }

  ResetRawPointCloud();  
  init_xyz_lut();
  inited_ = true;

  return true;
}

void Parser::ResetRawPointCloud() {
  raw_pointcloud_out_ = raw_pointcloud_pool_.at(pool_index_);
  ADEBUG << "pool index:" << pool_index_;
  raw_pointcloud_out_->Clear();
  raw_pointcloud_out_->mutable_point()->Reserve(70000);
  pool_index_ = (pool_index_ + 1) % pool_size_;
}

bool Parser::Parse(const std::shared_ptr<OusterScan>& scan) {
  ResetRawPointCloud();
  bool is_end = false;
  auto pf = OS1::get_format(info_.format);
  for (int i = 0; i < scan->firing_pkts_size(); ++i) {
    const auto& pkt = scan->firing_pkts(i);
    uint8_t* data =
        reinterpret_cast<uint8_t*>(const_cast<char*>(pkt.data().c_str()));
    ParseLidarPacket(data, pkt.data().size(), pf, &is_end);
  }
  PublishRawPointCloud(scan->header().sequence_num());
  return true;
}

void Parser::Parse(const uint8_t* data, int size, 
                    const OS1::packet_format& pf, bool* is_end) {
  bool t_is_end = false;
  ParseLidarPacket(data, size, pf, &t_is_end);
  ++packet_nums_;
  *is_end = CheckIsEnd(t_is_end);
  if (*is_end == false) {
    return;
  }
  packet_nums_ = 0;
  PublishRawPointCloud();
  ResetRawPointCloud();
}

void Parser::ParseIMU(const uint8_t* buf, const OS1::packet_format& pf)
{
  ParseImuPacket(buf, pf);
}

bool Parser::CheckIsEnd(bool is_end) {
  if (packet_nums_ >= max_packets_) {
    AWARN << "over max packets, packets:" << packet_nums_
          << ", max packets:" << max_packets_;
    return true;
  }
  if (is_end && packet_nums_ < min_packets_) {
    AWARN << "receive too less packets:" << packet_nums_ << ", not end"
          << ", min packets:" << min_packets_;
    return false;
  }
  return is_end;
}

void Parser::PublishRawPointCloud(int seq) {
  int size = raw_pointcloud_out_->point_size();
  if (size == 0) {
    AWARN << "All points size is NAN! Please check ouster:" << conf_.model();
    return;
  }

  raw_pointcloud_out_->mutable_header()->set_sequence_num(seq_index_++);
  if (seq > 0) {
    raw_pointcloud_out_->mutable_header()->set_sequence_num(seq);
  }
  raw_pointcloud_out_->mutable_header()->set_frame_id(conf_.frame_id());
  raw_pointcloud_out_->mutable_header()->set_timestamp_sec(
      cyber::Time().Now().ToSecond());
  raw_pointcloud_out_->set_height(1);
  raw_pointcloud_out_->set_width(size);
  const auto timestamp =
      raw_pointcloud_out_->point(static_cast<int>(size) - 1).timestamp();
  raw_pointcloud_out_->set_measurement_time(static_cast<double>(timestamp) /
                                            1e9);
  raw_pointcloud_out_->mutable_header()->set_lidar_timestamp(timestamp);
  raw_pointcloud_writer_->Write(raw_pointcloud_out_);
}

void Parser::Stop() {
  running_.store(false);
  if (online_calibration_thread_.joinable()) {
    online_calibration_thread_.join();
  }
}

void Parser::LoadSenseInfo(OS1::sensor_info& info)
{
  AINFO << "Start LoadCalibrationThread";
  AINFO << "Connecting to sensor at " << conf_.hostname();
  AINFO << "Sending data to " << conf_.udp_dest_host()
        << ", lidar receive port: " << conf_.lidar_recv_port()
        << ", imu receive port: " << conf_.imu_recv_port()
        << ", using lidar_mode: " << conf_.lidar_mode();

  auto meta_file = conf_.metadata_file();
  if (!meta_file.size() && conf_.hostname().size()) 
  {
    meta_file = conf_.hostname() + ".json";
  }
  AINFO << "metadata file is " << meta_file;


  auto cli = OS1::init_client(conf_.hostname(), conf_.udp_dest_host(),
                              OS1::lidar_mode_of_string(conf_.lidar_mode()),
                              conf_.lidar_recv_port(), conf_.imu_recv_port());
  if (!cli) {
      AERROR << "Failed to initialize sensor at: " << conf_.hostname();
      return;
  }
  else
  {
      OS1::get_udp_sockers(lidar_socker_, imu_socker_, 
                      socket_number_, *cli);
  }
  AINFO << "Sensor reconfigured successfully, waiting for data...";

  // write metadata file to cwd (usually ~/.ros)
  auto metadata = OS1::get_metadata(*cli);
  OS1::write_metadata(meta_file, metadata);

  // populate sensor info
  info = OS1::parse_metadata(metadata);
  OS1::populate_metadata_defaults(info, "");

  AINFO << info.prod_line.c_str() << " sn: " 
        << info.sn.c_str() <<" firmware rev: "
        << info.fw_rev.c_str();
  
}

void Parser::get_sensor_params(int &lidar_socker, int &imu_socker, 
                      int &socket_number, OS1::sensor_info& info) {
  lidar_socker = lidar_socker_;
  imu_socker = imu_socker_;
  socket_number = socket_number_;
  info = info_;
}

Parser* ParserFactory::CreateParser(const std::shared_ptr<Node>& node,
                                    const Config& conf) {
  if (conf.model() == OUSTER128) {
    AINFO << "creat OUSTER128 parser";
    return new Ouster128Parser(node, conf);
  }

  AERROR << "only support OUSTER128 | OUSTER64";
  return nullptr;
}

}  // namespace ouster
}  // namespace drivers
}  // namespace apollo
