cc_library(
    name = "slam_core",
    hdrs = [
        "camera_model.hpp",
        "landmark.hpp",
        "world.hpp",
        "pose2d.hpp",
        "sensor.hpp",
        "camera_sensor.hpp",
        "robot.hpp",
        "localizer.hpp",
    ],
    strip_include_prefix = "",  # expose headers relative to this package
    visibility = ["//visibility:public"],
    copts = [
        "-Wall",
        "-Wextra",
        "-Werror",
    ],
    deps = [
        "@sophus//:sophus",
    ],
)


cc_library(
    name = "navlog",
    hdrs = [
        "navlog.hpp",
    ],
    strip_include_prefix = "",
    visibility = ["//visibility:public"],
    copts = [
        "-Wall",
        "-Wextra",
        "-Werror",
    ],
    deps = [
        ":slam_core",      # uses Robot, World, CameraModel, etc.
        "@sophus//:sophus",
    ],
)
