workspace(name = "visual_slam_2d")

load("@bazel_tools//tools/build_defs/repo:http.bzl", "http_archive")

http_archive(
    name = "com_google_googletest",
    urls = ["https://github.com/google/googletest/archive/refs/tags/v1.14.0.zip"],
    strip_prefix = "googletest-1.14.0",
)

load("@com_google_googletest//:bazel/repositories.bzl", "googletest_repositories")
googletest_repositories()

load("@com_google_googletest//:bazel/repositories.bzl", "googletest_deps")
googletest_deps()
