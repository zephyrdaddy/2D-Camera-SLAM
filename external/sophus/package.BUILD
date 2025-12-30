package(default_visibility = ["//visibility:public"])

# Gather all header files
filegroup(
    name = "sophus_headers",
    srcs = glob([
        "sophus/**/*.hpp",
    ]),
)

# Main Sophus library target
cc_library(
    name = "sophus",
    hdrs = [":sophus_headers"],
    includes = ["."],  # Include from the root of the repo
    deps = [
        "@eigen//:eigen",  # Eigen dependency
        "@fmt//:fmt",
    ],
    linkstatic = True,
)

# Alias target for convenience
alias(
    name = "Sophus",
    actual = ":sophus",
)
