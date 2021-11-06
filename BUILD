load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

cc_binary(
    name = "libouster_driver_component.so",
    linkstatic = False,
    linkopts = [
    	"-shared",
        "-L/apollo/modules/drivers/ouster/lib",
        "-Wl,-rpath=/apollo/modules/drivers/ouster/lib",      
        "-louster_driver",
    ],
    deps = [
        "//cyber",
        "//modules/common/util",
        "//modules/drivers/ouster/proto:ouster_proto",
        "//modules/drivers/proto:sensor_proto",
    ],
)

cpplint()
