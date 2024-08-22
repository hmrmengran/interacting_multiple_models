load("@rules_cc//cc:defs.bzl", "cc_binary", "cc_library", "cc_test")
load("//tools/install:install.bzl", "install")
load("//tools:cpplint.bzl", "cpplint")

package(default_visibility=["//visibility:public"])

cc_library(
    name = "kalmanfilter",
    hdrs = ["kalmanfilter.h"],
    srcs = ["kalmanfilter.cc"],
    includes = ["./"],
    deps = [
        "//cyber",
        "@eigen",
        ],
    visibility = ["//visibility:public"],
    # copts = ["-std=c++17"],
)

# cc_library(
#     name = "immkf",
#     hdrs = ["immkf.h"],
#     srcs = ["immkf.cc"],
#     includes = ["./"],
#     deps = [
#         ":kalmanfilter",
#         "@eigen",
#         ],
#     visibility = ["//visibility:public"],
#     copts = ["-std=c++17"],
# )

# cc_test(
#     name = "imm_test",
#     srcs = ["imm_test.cc"],
#     deps = [
#         ":immkf",
#         ":kalmanfilter",
#         "@com_google_googletest//:gtest_main",
#     ],
#     linkopts = ["-lm"],
#     # copts = ["-std=c++17"],
# )

# filegroup(
#     name = "runtime_data",
#     srcs = glob([
#         "conf/*.txt",
#         "conf/*.yaml",
#         "dag/*.dag",
#         "launch/*.launch"
#     ]),
# )

# install(
#     name="install",
#     data=[
#         ":runtime_data",
#     ],
#     targets=[
#         ":distortion_correction.so",
#     ],
#     deps=[
#         "//cyber:install",
#     ],
# )

# cpplint()
