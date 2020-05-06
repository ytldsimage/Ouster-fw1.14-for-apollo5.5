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
#include <poll.h>
#include <netdb.h>

#include "modules/drivers/ouster/driver.h"
#include "modules/drivers/ouster/compat.h"

namespace apollo {
namespace drivers {
namespace ouster {

bool OusterDriver::Init() {
  if (node_ == nullptr) {
    AERROR << "node is nullptr";
    return false;
  }
  scan_writer_ = node_->CreateWriter<OusterScan>(conf_.scan_channel());
  if (scan_writer_ == nullptr) {
    AERROR << "writer:" << conf_.scan_channel()
           << " create error, check cyber is inited.";
    return false;
  }
   
  if (!parser_->Init()) {
    AERROR << "parser init error";
    return false;
  }

  tz_second_ = conf_.time_zone() * 3600;

  parser_->get_sensor_params(socket_lidar_, socket_imu_, 
                          socket_count_, sensor_info_);
  if ((socket_count_ < 1) || (socket_count_ > 2)){
    AERROR << "socker number[" << socket_count_ << "] is error";
    return false;
  }
  // else if (socket_lidar_ > 0)
  // {
  //   AINFO << "socket[" << socket_lidar_ << "] only receive lidar date.";
  //   socket_count_ = 1;
  // }
  // else
  // {
  //   AERROR << "other case,undo";
  //   return false;
  // }

  auto pf = OS1::get_format(sensor_info_.format);
  imu_packet_size_ = pf.imu_packet_size;
  if (conf_.model() == OUSTER128) {
    pkt_buffer_capacity_ = OUSTER128_MAX_PACKETS * 10;
    lidar_packet_size_ = pf.lidar_packet_size;    
  } else {
    AERROR << "Not support model:" << conf_.model();
    return false;
  }

  AINFO << "packet buffer capacity:" << pkt_buffer_capacity_;
  pkt_buffer_.resize(pkt_buffer_capacity_);
  for (int i = 0; i < pkt_buffer_capacity_; ++i) {
    pkt_buffer_[i] = std::make_shared<OusterPacket>();
  }

  scan_buffer_.resize(scan_buffer_size_);
  for (int i = 0; i < scan_buffer_size_; ++i) {
    scan_buffer_[i] = std::make_shared<OusterScan>();
    if (scan_buffer_[i] == nullptr) {
      AERROR << "make scan buffer error";
      return false;
    }
  }

  poll_thread_ = std::thread(&OusterDriver::PollThread, this);
  process_thread_ = std::thread(&OusterDriver::ProcessThread, this);
  return true;
}

void OusterDriver::PollThread() {
  AINFO << "Poll thread start";
  while (running_) {

    auto state = poll_client(socket_lidar_, socket_imu_);
    if (state == OS1::EXIT) {
        AINFO << "poll_client: caught signal, exiting";
        continue;
    }
    if (state & OS1::CLIENT_ERROR) {
        AERROR << "poll_client: returned error";
        continue;
    }
    if (state & OS1::LIDAR_DATA) {
        receive_data(socket_lidar_, lidar_packet_size_);    
    }
    if (state & OS1::IMU_DATA) {
        receive_data(socket_imu_, imu_packet_size_);
    }

  }
    
}

void OusterDriver::receive_data(int fd, size_t recv_max_len, 
                                int max_cnt) {
  int count = 0;
  int64_t last_size = 0;
  ADEBUG << "fd [" << fd << "] receive " << recv_max_len;
  while (count < max_cnt) {
    std::shared_ptr<OusterPacket>& pkt = pkt_buffer_[pkt_index_];
    int64_t n = OS1::recv_non_fixed(fd, &pkt->data[0], recv_max_len);
    ADEBUG << " fd [" << fd << "]  received packet size =  " << n; 
    if ((n > 0) && (n < (int64_t)(recv_max_len + 1)))
    {
      pkt->size = n;
      {
        std::lock_guard<std::mutex> lck(packet_mutex_);
        pkt_queue_.push_back(pkt);
        packet_condition_.notify_all();
      }
      pkt_index_ = (pkt_index_ + 1) % pkt_buffer_capacity_;
      count++;
      last_size = n;
    } else {
      if ((count > 0) || (n > (int64_t)recv_max_len))
        ADEBUG << "received [p-" << count << "]  packets " << last_size;     
      break;
    }
  }
}

void OusterDriver::ProcessIMU(const OusterPacket& pkt,
                              const OS1::packet_format& pf) {
  if (pkt.size != imu_packet_size_) {
    return;
  }
  parser_->ParseIMU(pkt.data, pf);
}

void OusterDriver::ProcessThread() {
  std::shared_ptr<OusterPacket> pkt = nullptr;
  bool is_end = false;
  int seq = 0;
  auto pf = OS1::get_format(sensor_info_.format);
  while (running_) {
    {
      std::unique_lock<std::mutex> lck(packet_mutex_);
      if (pkt_queue_.empty()) {
        packet_condition_.wait(lck);
      }
      if (pkt_queue_.empty()) {
        // exit notify empty
        continue;
      }
      pkt = pkt_queue_.front();
      pkt_queue_.pop_front();
    }

    if (pkt->size == imu_packet_size_) {
      ProcessIMU(*pkt, pf);
      continue;
    }
    scan_buffer_[index_]->add_firing_pkts()->set_data(pkt->data, pkt->size);
    parser_->Parse(pkt->data, pkt->size, pf, &is_end);
    if (is_end && scan_buffer_[index_]->firing_pkts_size() > 0) {
      is_end = false;
      auto now = apollo::cyber::Time().Now();
      scan_buffer_[index_]->mutable_header()->set_timestamp_sec(now.ToSecond());
      scan_buffer_[index_]->mutable_header()->set_frame_id(conf_.frame_id());
      scan_buffer_[index_]->mutable_header()->set_sequence_num(seq++);
      scan_buffer_[index_]->set_model(conf_.model());
      scan_buffer_[index_]->set_basetime(0);
      scan_writer_->Write(scan_buffer_[index_]);
      ADEBUG << "publish scan size:" << scan_buffer_[index_]->firing_pkts_size()
            << ", index:" << index_;
      ++index_;
      index_ = index_ % scan_buffer_size_;
      scan_buffer_[index_]->Clear();
    }
  }
}


OS1::client_state OusterDriver::poll_client(int lidar_fd, int imu_fd,
                                                  const int timeout_sec) {
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(lidar_fd, &rfds);
    FD_SET(imu_fd, &rfds);

    timeval tv;
    tv.tv_sec = timeout_sec;
    tv.tv_usec = 0;

    int max_fd = std::max(lidar_fd, imu_fd);

    int retval = select(max_fd + 1, &rfds, NULL, NULL, &tv);

    OS1::client_state res = OS1::client_state(0);

    if (!OS1::socket_valid(retval) && OS1::socket_exit()) {
        res = OS1::EXIT;
    } else if (!OS1::socket_valid(retval)) {
        AERROR << "select: " << OS1::socket_get_error();
        res = OS1::client_state(res | OS1::CLIENT_ERROR);
    } else if (retval) {
        if (FD_ISSET(lidar_fd, &rfds)) res = OS1::client_state(res | OS1::LIDAR_DATA);
        if (FD_ISSET(imu_fd, &rfds)) res = OS1::client_state(res | OS1::IMU_DATA);
    }

    return res;
}

}  // namespace ouster
}  // namespace drivers
}  // namespace apollo
