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

#ifndef LIDAR_OUSTER_SRC_TYPE_DEFS_H_
#define LIDAR_OUSTER_SRC_TYPE_DEFS_H_

#include "modules/drivers/ouster/const_var.h"

namespace apollo {
namespace drivers {
namespace ouster {

typedef struct OusterPacket {
  double stamp;
  uint8_t data[OS_128_DATA_PACKET_SIZE+1];
  uint32_t size;
} OusterPacket;

}  // namespace ouster
}  // namespace drivers
}  // namespace apollo

#endif
