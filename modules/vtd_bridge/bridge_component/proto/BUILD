load("//tools:apollo_package.bzl", "apollo_package")
load("//tools:cpplint.bzl", "cpplint")
load("//tools/proto:proto.bzl", "proto_library")

package(default_visibility = ["//visibility:public"])

proto_library(
    name = "bridge_component_config_proto",
    srcs = ["bridge_component_config.proto"],
)

apollo_package()

cpplint()
