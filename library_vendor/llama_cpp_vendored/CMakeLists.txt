cmake_minimum_required(VERSION 3.15)
project(llama_cpp_vendored)

find_package(ament_cmake REQUIRED)

include(GNUInstallDirs)

option(USE_CUDA "Build llama.cpp with CUDA support?" OFF)

if (USE_CUDA)
    set(CMAKE_CUDA_ARCHITECTURES native)
    set(CMAKE_CUDA_COMPILER /usr/local/cuda/bin/nvcc)
endif ()

set(GGML_CUDA_F16 ${USE_CUDA})
set(GGML_CUDA ${USE_CUDA})
set(GGML_NATIVE ON)
set(GGML_AVX512 OFF)
set(LLAMA_BUILD_SERVER ON)
set(LLAMA_BUILD_EXAMPLES ON)
set(BUILD_SHARED_LIBS OFF)

add_subdirectory(llama_cpp)
#NOTE: llama.cpp defines installation of the convert script, but we want to rename the file,
# so we define the install again ourselves anyway
install(
        FILES llama_cpp/convert_hf_to_gguf.py
        PERMISSIONS
        OWNER_READ
        OWNER_WRITE
        OWNER_EXECUTE
        GROUP_READ
        GROUP_EXECUTE
        WORLD_READ
        WORLD_EXECUTE
        DESTINATION ${CMAKE_INSTALL_BINDIR}
        RENAME llama-convert_hf_to_gguf.py)

ament_package()
