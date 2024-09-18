load("@rules_proto//proto:defs.bzl", "proto_library")
load("@rules_cc//cc:defs.bzl", "cc_proto_library")
load("//tools:python_rules.bzl", "py_proto_library")

package(default_visibility = ["//visibility:public"])

#seyond_config_proto
py_proto_library(
    name = "imm_config_py_proto",
    deps = [
        ":imm_config_proto",
    ],
)

cc_proto_library(
    name = "imm_config_cc_proto",
    deps = [
        ":imm_config_proto",
    ],
)

proto_library(
    name = "imm_config_proto",
    srcs = [
        "imm.proto",
    ],
    deps = [
        "//modules/omnisense/common/proto:pose_proto"
    ],
)


