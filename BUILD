load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libouster_driver_component.so",
    linkopts = ["-shared"],
    linkstatic = False,
    deps = [":driver"],
)

cc_library(
    name = "driver",
    srcs = [
        "jsoncpp.cpp",
        "compat.cc",
        "os1.cc",
        "component.cc",
        "driver.cc",
        "parser.cc",
        "ouster128_parser.cc",
    ],
    hdrs = [
        "json/json.h",
        "json/json-forwards.h",
        "version.h",
        "impl/os1_packet_impl.h",
        "os1_packet.h",
        "compat.h",
        "lidar_scan.h",
        "os1.h",
        "os1_impl.h",
        "const_var.h",
        "driver.h",
        "ouster_driver_component.h",
        "ouster_convert_component.h",
        "parser.h",
        "type_defs.h",

    ],
    copts = ['-DMODULE_NAME=\\"ouster\\"'],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/ouster/proto:ouster_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
