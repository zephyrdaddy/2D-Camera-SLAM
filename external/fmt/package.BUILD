cc_library(
    name = "fmt",
    # srcs = glob(["src/*.cc"]),
    srcs = ["src/format.cc", "src/os.cc"],  # explicitly list source files
    hdrs = glob(["include/fmt/**/*.h"]),
    includes = ["include"],
    visibility = ["//visibility:public"],
    copts = ["-std=c++17"],  # upgraded from C++14

)