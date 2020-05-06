load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "ouster_proto",
    deps = [
        ":sensor_ouster_proto",
    ],
)

cc_proto_library(
    name = "sensor_ouster_proto",
    deps = [
        ":ouster_proto_lib",
    ],
)

proto_library(
    name = "ouster_proto_lib",
    srcs = [
        "config.proto",
        "ouster.proto",
    ],
    deps = [
        "//modules/common/proto:header_proto_lib",
    ],
)

cpplint()
