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

#include <Eigen/Eigen>
#include <chrono>
#include <cmath>
#include <iterator>
#include <utility>
#include <vector>

#include "modules/drivers/ouster/parser.h"

namespace apollo {
namespace drivers {
namespace ouster {

/**
 * Unit of range from OS1 packet, in metres.
 */
constexpr double range_unit_t = 0.001;  // m

Ouster128Parser::Ouster128Parser(const std::shared_ptr<Node> &node,
                             const Config &conf)
    : Parser(node, conf) {}


Ouster128Parser::~Ouster128Parser() {}

void Ouster128Parser::init_xyz_lut() {
  pointcloud_w_ = info_.format.columns_per_frame;
  pointcloud_h_ = info_.format.pixels_per_column;
  AINFO << "init OS1-128, make_xyz, w = " << pointcloud_w_   
        <<", h = " << pointcloud_h_;
  xyz_lut_ = OS1::make_xyz_lut(pointcloud_w_, pointcloud_h_, range_unit_t,
  						info_.lidar_origin_to_beam_origin_mm,
  						info_.beam_azimuth_angles, info_.beam_altitude_angles);
  
}

void Ouster128Parser::ParseLidarPacket(const uint8_t *buf, const int len,
									const OS1::packet_format& pf, bool *is_end) {
  const int W = pointcloud_w_;
  const int H = pointcloud_h_;
  int next_m_id{pointcloud_w_};
  constexpr std::chrono::nanoseconds invalid_ts(-1LL);
  std::chrono::nanoseconds scan_ts(invalid_ts);

  *is_end = false;
  const uint8_t* packet_buf = (const uint8_t*)buf;  
  for (int icol = 0; icol < pf.columns_per_packet; icol++) {
      const uint8_t* col_buf = pf.nth_col(icol, packet_buf);
      const uint16_t m_id = pf.col_measurement_id(col_buf);
      const uint16_t f_id = pf.col_frame_id(col_buf);
      const std::chrono::nanoseconds ts(pf.col_timestamp(col_buf));
      if (2050 == first_m_id_)  first_m_id_ = m_id;
      last_m_id_ = m_id;
      const bool valid = pf.col_valid(col_buf) == 0xffffffff;

      // drop invalid / out-of-bounds data in case of misconfiguration
      if (!valid || m_id >= W || f_id + 1 == cur_f_id_) 
      {
          AINFO << "O64Warning drop invalid: valid = " << std::boolalpha << valid 
                <<", m_id = " << m_id << ", f_id = " << f_id << ", cur_f_id_ = " << cur_f_id_;
          continue;
      }

      if (f_id != cur_f_id_) {
          // if not initializing with first packet
          if (scan_ts != invalid_ts) {
              // apollo::drivers::PointXYZIT* point_new = pc->add_point();
              // point_new->set_x(nan);
              // point_new->set_y(nan);
              // point_new->set_z(nan);
              // point_new->set_timestamp(ts.count());
              // point_new->set_intensity(0);
              ADEBUG << "O64Warning set nan1 valid = " << std::boolalpha << valid 
                <<", m_id = " << m_id << ", f_id = " << f_id << ", cur_f_id_ = " << cur_f_id_;
          }

          // start new frame
          ADEBUG << "O64Debug,start new frame, scan_ts= " 
                << scan_ts.count() << ", ts_= " << ts.count()
                 <<", m_id = " << m_id << ", f_id = " 
                 << f_id << ", cur_f_id_ = " << cur_f_id_;
          scan_ts = ts;
          next_m_id = 0;
          cur_f_id_ = f_id;
          *is_end = true;

      }

      // zero out missing columns if we jumped forward
      if (m_id >= next_m_id) {
          // apollo::drivers::PointXYZIT* point_new = pc->add_point();
          // point_new->set_x(nan);
          // point_new->set_y(nan);
          // point_new->set_z(nan);
          // point_new->set_timestamp(ts.count());
          // point_new->set_intensity(0);
          next_m_id = m_id + 1;
          ADEBUG << "O64Warning set nan2 valid = " << std::boolalpha << valid 
                <<", m_id = " << m_id << ", f_id = " << f_id << ", cur_f_id_ = " << cur_f_id_;
      }

      // index of the first point in current packet
      for (uint8_t ipx = 0; ipx < H; ipx++) {
          const uint8_t* px_buf = pf.nth_px(ipx, col_buf);
          uint32_t r = pf.px_range(px_buf);

          PointXYZIT *point = raw_pointcloud_out_->add_point();
          
          point->set_timestamp(ts.count());

          point->set_x((float)((double)r * (xyz_lut_.direction)(ipx * W + m_id, 0))+(float)(xyz_lut_.offset)(ipx * W + m_id, 0));
          point->set_y((float)((double)r * (xyz_lut_.direction)(ipx * W + m_id, 1))+(float)(xyz_lut_.offset)(ipx * W + m_id, 1));
          point->set_z((float)((double)r * (xyz_lut_.direction)(ipx * W + m_id, 2))+(float)(xyz_lut_.offset)(ipx * W + m_id, 2));

          point->set_intensity(pf.px_signal_photons(px_buf));
      }
  }  
}

void Ouster128Parser::ParseImuPacket(const uint8_t* buf, const OS1::packet_format& pf)
{
	n_imu_packets_++;

    uint64_t imu_ts = pf.imu_gyro_ts(buf);
    float imu_av_z = pf.imu_av_z(buf);
    float imu_la_y = pf.imu_la_y(buf);
    if (n_imu_packets_ % 50 == 0) 
    {
    	ADEBUG <<"IMU[" << std::setw(15) << n_imu_packets_ << "]" 
    		  << std::setw(15) << imu_av_z
              << std::setw(15) << imu_la_y << std::setw(20) << imu_ts;
    }
}


}  // namespace ouster
}  // namespace drivers
}  // namespace apollo

