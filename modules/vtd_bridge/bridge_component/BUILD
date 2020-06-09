load("//tools:apollo_package.bzl", "apollo_component", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility = ["//visibility:public"])

filegroup(
    name = "runtime_data",
    srcs = glob([
        "dag/**",
        "launch/**",
        "conf/**",
    ]),
)

apollo_component(
    name = "libbridge_component.so",
    srcs = [
        "bridge_component.cc",
        "utils/DecodeMsg.cc",
        "utils/RDBHandler.cc",
        "utils/Socket.cc",
    ],
    hdrs = [
        "bridge_component.h",
        "utils/DecodeMsg.h",
        "utils/RDBHandler.h",
        "utils/Socket.h",
        "utils/viRDBIcd.h",
        "utils/viRDBTypes.h",
    ],
    copts = [
        "-DMODULE_NAME=\\\"vtd_bridge\\\"",
    ],
    deps = [
        "//cyber",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common/math",
        "//modules/common/util:util_tool",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/perception_msgs:perception_obstacle_cc_proto",
        "//modules/common_msgs/perception_msgs:traffic_light_detection_cc_proto",
        "//modules/common_msgs/prediction_msgs:prediction_obstacle_cc_proto",
        "//modules/common_msgs/transform_msgs:transform_cc_proto",
        "//modules/localization/common:localization_gflags",
        "//modules/vtd_bridge/bridge_component/proto:bridge_component_config_cc_proto",
        "@eigen",
    ],
)

apollo_package()

cpplint()
