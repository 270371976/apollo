
package(default_visibility = ["//visibility:public"])

CONTROL_COPTS = ['-DMODULE_NAME=\\"control\\"']

cc_library(
    name = "control_gflags",
    srcs = ["control_gflags.cc"],
    hdrs = ["control_gflags.h"],
    copts = CONTROL_COPTS,
    deps = [
        "@com_github_gflags_gflags//:gflags",
    ],
)

cc_library(
    name = "hysteresis_filter",
    srcs = ["hysteresis_filter.cc"],
    hdrs = ["hysteresis_filter.h"],
    copts = CONTROL_COPTS,
)


cc_library(
    name = "interpolation_1d",
    srcs = ["interpolation_1d.cc"],
    hdrs = ["interpolation_1d.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "@eigen",
    ],
)

cc_library(
    name = "interpolation_2d",
    srcs = ["interpolation_2d.cc"],
    hdrs = ["interpolation_2d.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
    ],
)

cc_library(
    name = "leadlag_controller",
    srcs = ["leadlag_controller.cc"],
    hdrs = ["leadlag_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/proto:leadlag_conf_cc_proto",
    ],
)

cc_library(
    name = "mrac_controller",
    srcs = ["mrac_controller.cc"],
    hdrs = ["mrac_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/common_msgs/config_msgs:vehicle_config_cc_proto",
        "//modules/common/math",
        "//modules/common/status",
        "//modules/control/proto:mrac_conf_cc_proto",
        "@com_google_absl//:absl",
        "@eigen",
    ],
)

cc_library(
    name = "pid_controller",
    srcs = ["pid_controller.cc"],
    hdrs = ["pid_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        "//cyber",
        "//modules/control/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "pid_BC_controller",
    srcs = ["pid_BC_controller.cc"],
    hdrs = ["pid_BC_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "pid_IC_controller",
    srcs = ["pid_IC_controller.cc"],
    hdrs = ["pid_IC_controller.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":pid_controller",
        "//cyber",
        "//modules/common/math",
        "//modules/control/proto:pid_conf_cc_proto",
    ],
)

cc_library(
    name = "trajectory_analyzer",
    srcs = ["trajectory_analyzer.cc"],
    hdrs = ["trajectory_analyzer.h"],
    copts = CONTROL_COPTS,
    deps = [
        ":control_gflags",
        "//cyber",
        "//modules/common/math",
        "//modules/common_msgs/basic_msgs:pnc_point_cc_proto",
        "//modules/common/vehicle_state:vehicle_state_provider",
        "//modules/common_msgs/planning_msgs:planning_cc_proto",
        "@eigen",
    ],
)

cc_library(
    name = "dependency_injector",
    hdrs = ["dependency_injector.h"],
    deps = [
        "//modules/common/vehicle_state:vehicle_state_provider",
    ],
)

cc_library(
    name = "common",
    copts = CONTROL_COPTS,
    deps = [
        ":control_gflags",
        ":hysteresis_filter",
        ":interpolation_1d",
        ":interpolation_2d",
        ":leadlag_controller",
        ":mrac_controller",
        ":pid_controller",
        ":pid_BC_controller",
        ":pid_IC_controller",
        ":trajectory_analyzer",
        ":dependency_injector",
    ],
)





