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


#ifndef LIDAR_OUSTER_CONST_VAR_H_
#define LIDAR_OUSTER_CONST_VAR_H_

namespace apollo {
namespace drivers {
namespace ouster {

// ouster 128
const int OUSTER128_MAX_PACKETS = 150;
const int OUSTER128_MIN_PACKETS = 15;

const int OS_128_DATA_PACKET_SIZE = 24896;
const int OS_128_IMU_PACKET_SIZE = 48;

}  // namespace ouster
}  // namespace drivers
}  // namespace apollo

#endif
