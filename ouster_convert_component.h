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

#ifndef LIDAR_OUSTER_OUSTER_CONVERT_COMPONENT_H_
#define LIDAR_OUSTER_OUSTER_CONVERT_COMPONENT_H_

#include <list>
#include <memory>
#include <string>
#include <thread>

#include "cyber/cyber.h"
#include "modules/drivers/ouster/const_var.h"
#include "modules/drivers/ouster/parser.h"
#include "modules/drivers/ouster/type_defs.h"

#include "modules/drivers/ouster/proto/config.pb.h"
#include "modules/drivers/ouster/proto/ouster.pb.h"

namespace apollo {
namespace drivers {
namespace ouster {

using apollo::cyber::Component;
using apollo::drivers::ouster::OusterScan;

class OusterConvertComponent : public Component<OusterScan> {
 public:
  ~OusterConvertComponent() {}
  bool Init() override {
    if (!GetProtoConfig(&conf_)) {
      AERROR << "load config error, file:" << config_file_path_;
      return false;
    }

    AINFO << "conf:" << conf_.DebugString();
    Parser* parser = ParserFactory::CreateParser(node_, conf_);
    if (parser == nullptr) {
      AERROR << "create parser error";
      return false;
    }
    parser_.reset(parser);

    if (!parser_->Init()) {
      return false;
    }
    AINFO << "OusterConvertComponent init success";
    return true;
  }

  bool Proc(const std::shared_ptr<OusterScan>& scan) override {
    return parser_->Parse(scan);
  }

 private:
  std::shared_ptr<Parser> parser_;
  Config conf_;
};

CYBER_REGISTER_COMPONENT(OusterConvertComponent)

}  // namespace ouster
}  // namespace drivers
}  // namespace apollo

#endif
