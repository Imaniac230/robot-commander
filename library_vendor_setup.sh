#!/bin/bash

function scriptRootDir() {
  #https://stackoverflow.com/questions/59895/how-can-i-get-the-source-directory-of-a-bash-script-from-within-the-script-itsel/246128
  # shellcheck disable=SC2005
  echo "$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
}

libraryVendorPath="$(scriptRootDir)/library_vendor"
inputPath="$libraryVendorPath/input"
barkCppPath="$libraryVendorPath/bark_cpp_vendored/bark_cpp"

echo -e "\n\nDownloading external libraries ...\n"
vcs import "$libraryVendorPath" < "$libraryVendorPath"/libraries.repos || { echo "Failed to download external libraries !"; exit 1; }

echo -e "\n\nInitializing submodules for 'bark.cpp' ...\n"
pushd "$barkCppPath" || { echo "ERROR: Could not push into the '$barkCppPath' directory."; exit 1; }
git submodule update --init --recursive
popd || { echo "ERROR: Could not pop out of the '$barkCppPath' directory."; exit 1; }

echo -e "\n\nDownloading dependencies for 'input' ...\n"
vcs import "$inputPath" < "$inputPath"/libraries.repos || { echo "Failed to download dualsense library !"; exit 1; }

echo -e "\n\nAll libraries downloaded.\n"
