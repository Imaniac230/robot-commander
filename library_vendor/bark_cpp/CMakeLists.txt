cmake_minimum_required(VERSION 3.15)
project(bark_cpp_vendored)

find_package(ament_cmake REQUIRED)

include(GNUInstallDirs)

option(USE_CUDA "Build bark.cpp with CUDA support?" OFF)

if (USE_CUDA)
    set(CMAKE_CUDA_ARCHITECTURES native)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
endif ()

set(GGML_CUDA_F16 ${USE_CUDA})
#TODO(cmake-options): must be defined as options for older cmake specified in bark.cpp -> cmake --help-policy CMP0077
#FIXME(bark-cmake): we can currently modify this directly in our fork of bark.cpp instead
option(GGML_CUBLAS "" ${USE_CUDA})
option(GGML_AVX512 "" OFF)
option(BUILD_SHARED_LIBS "" OFF)

add_subdirectory(bark_cpp)
#NOTE: overriding default binary names
set_target_properties(quantize PROPERTIES OUTPUT_NAME bark-quantize)
set_target_properties(server PROPERTIES OUTPUT_NAME bark-server)
set_target_properties(main PROPERTIES OUTPUT_NAME bark-main)
#NOTE: bark.cpp currently installs only the server example, so we define the remaining installs here ourselves
install(TARGETS quantize RUNTIME)
install(TARGETS main RUNTIME)
install(
        FILES bark_cpp/convert.py
        PERMISSIONS
        OWNER_READ
        OWNER_WRITE
        OWNER_EXECUTE
        GROUP_READ
        GROUP_EXECUTE
        WORLD_READ
        WORLD_EXECUTE
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        RENAME bark-convert.py)

ament_package()
