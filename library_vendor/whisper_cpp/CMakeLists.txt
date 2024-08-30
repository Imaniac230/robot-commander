cmake_minimum_required(VERSION 3.15)
project(whisper_cpp_vendored)

find_package(ament_cmake REQUIRED)

include(GNUInstallDirs)

option(USE_CUDA "Build whisper.cpp with CUDA support?" OFF)

if (USE_CUDA)
    set(CMAKE_CUDA_ARCHITECTURES native)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
endif ()

set(GGML_CUDA_F16 ${USE_CUDA})
#TODO(cmake-options): must be defined as options for older cmake specified in whisper.cpp -> cmake --help-policy CMP0077
option(WHISPER_CUDA "" ${USE_CUDA})
option(WHISPER_NO_AVX512 "" ON)
option(WHISPER_BUILD_EXAMPLES "" ON)
option(BUILD_SHARED_LIBS "" OFF)

add_subdirectory(whisper_cpp)
#NOTE: overriding default binary names
set_target_properties(quantize PROPERTIES OUTPUT_NAME whisper-quantize)
set_target_properties(server PROPERTIES OUTPUT_NAME whisper-server)
set_target_properties(main PROPERTIES OUTPUT_NAME whisper-main)
#NOTE: whisper.cpp does not currently specify installation of examples and scripts, so we define it here ourselves
install(TARGETS quantize RUNTIME)
install(TARGETS server RUNTIME)
install(TARGETS main RUNTIME)
install(
        FILES whisper_cpp/models/convert-pt-to-ggml.py
        PERMISSIONS
        OWNER_READ
        OWNER_WRITE
        OWNER_EXECUTE
        GROUP_READ
        GROUP_EXECUTE
        WORLD_READ
        WORLD_EXECUTE
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        RENAME whisper-convert-pt-to-ggml.py)
install(
        FILES whisper_cpp/models/convert-h5-to-ggml.py
        PERMISSIONS
        OWNER_READ
        OWNER_WRITE
        OWNER_EXECUTE
        GROUP_READ
        GROUP_EXECUTE
        WORLD_READ
        WORLD_EXECUTE
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        RENAME whisper-convert-h5-to-ggml.py)

ament_package()
