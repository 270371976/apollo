

package(default_visibility = ["//visibility:public"])

cc_library(
    name = "control_component_lib",
    srcs = ["control_component.cc"],
    hdrs = ["control_component.h"],
    copts = ['-DMODULE_NAME=\\"control\\"'],
    deps = [
        "//cyber",
        "//modules/common/latency_recorder",
        "//modules/common/monitor_log",
        "//modules/common/util",
        "//modules/control/common",
        "//modules/control/controller",
        "//modules/common/adapters:adapter_gflags",
        "//modules/common_msgs/chassis_msgs:chassis_cc_proto",
        "//modules/common_msgs/control_msgs:control_cmd_cc_proto",
        "//modules/control/proto:control_conf_cc_proto",
        "//modules/common_msgs/control_msgs:pad_msg_cc_proto",
        "//modules/common_msgs/localization_msgs:localization_cc_proto",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "//modules/control/common:dependency_injector",
        "//modules/control/proto:preprocessor_cc_proto",
         "@com_github_gflags_gflags//:gflags",
    ],
    alwayslink = True
)

cc_binary(
    name = "libcontrol_component.so",
    linkshared = True,
    linkstatic = True,
    deps = [":control_component_lib"],
)



