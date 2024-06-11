#!/bin/bash

function scriptRootDir() {
  #https://stackoverflow.com/questions/59895/how-can-i-get-the-source-directory-of-a-bash-script-from-within-the-script-itsel/246128
  # shellcheck disable=SC2005
  echo "$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
}

useCuda=""
while [[ $useCuda != "OFF" ]] && [[ $useCuda != "ON" ]]; do
  read -rp "Attempt to build llama.cpp and whisper.cpp with CUDA support? [y/n] " useCuda
  useCuda=$(echo "$useCuda" | tr '[:upper:]' '[:lower:]')
  if [[ $useCuda == "n" ]] || [[ $useCuda == "no" ]]; then
    useCuda="OFF"
  elif [[ $useCuda == "y" ]] || [[ $useCuda == "yes" ]]; then
    useCuda="ON"
  fi
done

buildDirectory="build"

llamaCppPath="$(scriptRootDir)/libs/llama_cpp"
llamaCppOptions="-DLLAMA_CUDA=$useCuda -DLLAMA_CUDA_F16=$useCuda -DLLAMA_NATIVE=ON -DLLAMA_AVX512=OFF"

whisperCppPath="$(scriptRootDir)/libs/whisper_cpp"
whisperCppOptions="-DWHISPER_CUDA=$useCuda -DGGML_CUDA_F16=$useCuda -DWHISPER_NO_AVX512=ON"

barkCppPath="$(scriptRootDir)/libs/bark_cpp"
barkCppOptions="-DGGML_CUBLAS=$useCuda"

cudaOptions="-DCMAKE_CUDA_ARCHITECTURES=native -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc"

barkPath="$(scriptRootDir)/libs/bark"


echo -e "\nDownloading external libraries ...\n\n"
vcs import ./ < libraries.repos

echo -e "\nBuilding llama.cpp ...\n\n"
pushd "$llamaCppPath" || { echo "ERROR: Could not push into the '$llamaCppPath' directory."; exit 1; }
# shellcheck disable=SC2015
# this is desired behavior
mkdir -p "$buildDirectory" && cd "$buildDirectory" || { echo "Failed to setup the '$buildDirectory' build directory !"; exit 1; }
# shellcheck disable=SC2015
# shellcheck disable=SC2086
# this is desired behavior
cmake .. -G Ninja $llamaCppOptions $cudaOptions -DCMAKE_BUILD_TYPE=Release && ninja || { echo "Failed to build llama.cpp !"; exit 1; }
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
cmake .. -G Ninja $whisperCppOptions $cudaOptions -DCMAKE_BUILD_TYPE=Release && ninja || { echo "Failed to build whisper.cpp !"; exit 1; }
popd || { echo "ERROR: Could not pop out of the '$whisperCppPath' directory."; exit 1; }
echo -e "\nwhisper.cpp built successfully.\n"

echo -e "\nBuilding bark.cpp ...\n\n"
pushd "$barkCppPath" || { echo "ERROR: Could not push into the '$barkCppPath' directory."; exit 1; }
git submodule update --init --recursive
# shellcheck disable=SC2015
# this is desired behavior
mkdir -p "$buildDirectory" && cd "$buildDirectory" || { echo "Failed to setup the '$buildDirectory' build directory !"; exit 1; }
# shellcheck disable=SC2015
# shellcheck disable=SC2086
# this is desired behavior
cmake .. -G Ninja $barkCppOptions $cudaOptions -DCMAKE_BUILD_TYPE=Release && ninja || { echo "Failed to build bark.cpp !"; exit 1; }
popd || { echo "ERROR: Could not pop out of the '$barkCppPath' directory."; exit 1; }
echo -e "\nbark.cpp built successfully.\n"

echo -e "\nInstalling Bark\n\n"
pushd "$barkPath" || { echo "ERROR: Could not push into the '$barkPath' directory."; exit 1; }
pip install .
popd || { echo "ERROR: Could not pop out of the '$barkPath' directory."; exit 1; }
echo -e "\nBark installed successfully\n\n"

echo -e "\nAll libraries built successfully.\n"
