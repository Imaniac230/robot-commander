#!/bin/bash

function scriptRootDir() {
  #https://stackoverflow.com/questions/59895/how-can-i-get-the-source-directory-of-a-bash-script-from-within-the-script-itsel/246128
  # shellcheck disable=SC2005
  echo "$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
}

buildDirectory="build"

llamaCppPath="$(scriptRootDir)/libs/llama_cpp"
llamaCppOptions="-DLLAMA_CUDA=ON -DLLAMA_CUDA_F16=ON -DLLAMA_NATIVE=ON -DLLAMA_AVX512=OFF"

whisperCppPath="$(scriptRootDir)/libs/whisper_cpp"
whisperCppOptions="-DWHISPER_CUDA=ON -DWHISPER_NO_AVX512=ON"

cudaOptions="-DCMAKE_CUDA_ARCHITECTURES=native -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc"

barkPath="$(scriptRootDir)/libs/bark"

vcs import ./ < libraries.repos


echo -e "\nBuilding llama.cpp ...\n\n"
pushd "$llamaCppPath" || { echo "ERROR: Could not push into the '$llamaCppPath' directory."; exit 1; }
# shellcheck disable=SC2015
# this is desired behavior
mkdir -p "$buildDirectory" && cd "$buildDirectory" || { echo "Failed to setup the '$buildDirectory' build directory !"; exit 1; }
# shellcheck disable=SC2015
# shellcheck disable=SC2086
# this is desired behavior
cmake .. -G Ninja $llamaCppOptions $cudaOptions -DCMAKE_BUILD_TYPE=Release && ninja -j 16 || { echo "Failed to build llama.cpp !"; exit 1; }
popd || { echo "ERROR: Could not pop out of the '$llamaCppPath' directory."; exit 1; }
echo -e "\nllama.cpp built successfully.\n"

echo -e "\nBuilding whisper.cpp ...\n\n"
pushd "$whisperCppPath" || { echo "ERROR: Could not push into the '$whisperCppPath' directory."; exit 1; }
# shellcheck disable=SC2015
# this is desired behavior
mkdir -p "$buildDirectory" && cd "$buildDirectory" || { echo "Failed to setup the '$buildDirectory' build directory !"; exit 1; }
# shellcheck disable=SC2015
# shellcheck disable=SC2086
# this is desired behavior
cmake .. -G Ninja $whisperCppOptions $cudaOptions -DCMAKE_BUILD_TYPE=Release && ninja -j 16 || { echo "Failed to build whisper.cpp !"; exit 1; }
popd || { echo "ERROR: Could not pop out of the '$whisperCppPath' directory."; exit 1; }
echo -e "\nwhisper.cpp built successfully.\n"

echo -e "\nInstalling Bark\n\n"
pushd "$barkPath" || { echo "ERROR: Could not push into the '$barkPath' directory."; exit 1; }
pip install .
popd || { echo "ERROR: Could not pop out of the '$barkPath' directory."; exit 1; }
echo -e "\nBark installed successfully\n\n"

echo -e "\nAll libraries built successfully.\n"
