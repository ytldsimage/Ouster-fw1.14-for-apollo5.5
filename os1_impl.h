#pragma once

#include "modules/drivers/ouster/json/json.h"
#include "modules/drivers/ouster/compat.h"
#include "stdio.h"

namespace apollo {
namespace drivers {
namespace ouster {
namespace OS1 {

struct client {
    SOCKET lidar_fd;
    SOCKET imu_fd;
    int socket_number;
    std::string hostname;
    Json::Value meta;
    ~client() {
        socket_close(lidar_fd);
        socket_close(imu_fd);
    }
};

}  // namespace OS1
}  // namespace ouster
}  // namespace drivers
}  // namespace apollo
