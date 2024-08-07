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
llamaCppOptions="-DGGML_CUDA=$useCuda -DGGML_CUDA_F16=$useCuda -DGGML_NATIVE=ON -DGGML_AVX512=OFF"

whisperCppPath="$(scriptRootDir)/libs/whisper_cpp"
whisperCppOptions="-DWHISPER_CUDA=$useCuda -DGGML_CUDA_F16=$useCuda -DWHISPER_NO_AVX512=ON"

barkCppPath="$(scriptRootDir)/libs/bark_cpp"
barkCppOptions="-DGGML_CUBLAS=$useCuda -DGGML_CUDA_F16=$useCuda -DGGML_AVX512=OFF"

cudaOptions="-DCMAKE_CUDA_ARCHITECTURES=native -DCMAKE_CUDA_COMPILER=/usr/local/cuda/bin/nvcc"

barkPath="$(scriptRootDir)/libs/bark"

robotCommanderPath="$(scriptRootDir)"


echo -e "\nDownloading external libraries ...\n\n"
pushd "$robotCommanderPath" || { echo "ERROR: Could not push into the '$robotCommanderPath' directory."; exit 1; }
vcs import ./ < libraries.repos || { echo "Failed to download external all libraries !"; exit 1; }
popd || { echo "ERROR: Could not pop out of the '$robotCommanderPath' directory."; exit 1; }
echo -e "\nLibraries downloaded.\n"

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

echo -e "\nInstalling python Bark\n\n"
pushd "$barkPath" || { echo "ERROR: Could not push into the '$barkPath' directory."; exit 1; }
pip install . || { echo "Failed to install Bark !"; exit 1; }
popd || { echo "ERROR: Could not pop out of the '$barkPath' directory."; exit 1; }
echo -e "\nBark installed successfully\n\n"

echo -e "\nInstalling python robot-commander-library\n\n"
pushd "$robotCommanderPath" || { echo "ERROR: Could not push into the '$robotCommanderPath' directory."; exit 1; }
pip install . || { echo "Failed to install robot-commander-library !"; exit 1; }
popd || { echo "ERROR: Could not pop out of the '$robotCommanderPath' directory."; exit 1; }
echo -e "\nrobot-commander-library installed successfully\n\n"

echo -e "\nAll libraries built successfully.\n"

echo -e "\nSetting up paths\n\n"
# set_path_variable() "variable-name" "path"
function set_path_variable() {
 if grep -Fq "$1" ~/.bashrc; then
   # always replace whole path, escape all '/' characters in path with '\' for sed
   sed -i "/$1=/s/$1=[^\n]*/$1=${2//\//\\/}/" ~/.bashrc
 else
   echo "export $1=$2" >> ~/.bashrc
 fi
}

cp ~/.bashrc /tmp/
set_path_variable ROBOT_COMMANDER_LLAMA_CPP_PATH "$llamaCppPath"
set_path_variable ROBOT_COMMANDER_WHISPER_CPP_PATH "$whisperCppPath"
set_path_variable ROBOT_COMMANDER_BARK_CPP_PATH "$barkCppPath"
echo -e "Paths added to '~/.bashrc':\n<     original\n---\n>     new"
diff -s /tmp/.bashrc ~/.bashrc
