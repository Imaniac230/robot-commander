#!/bin/bash

function scriptRootDir() {
  #https://stackoverflow.com/questions/59895/how-can-i-get-the-source-directory-of-a-bash-script-from-within-the-script-itsel/246128
  # shellcheck disable=SC2005
  echo "$( cd "$( dirname "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )"
}

library_vendor_path="$(scriptRootDir)/library_vendor"
inputPath="$library_vendor_path/input"

echo -e "\nDownloading external libraries ...\n\n"
vcs import "$library_vendor_path" < "$library_vendor_path"/libraries.repos || { echo "Failed to download external libraries !"; exit 1; }
pushd "$library_vendor_path/bark_cpp/bark_cpp" || { echo "ERROR: Could not push into the '$library_vendor_path/bark_cpp/bark_cpp' directory."; exit 1; }
git submodule update --init --recursive
popd || { echo "ERROR: Could not pop out of the '$library_vendor_path/bark_cpp/bark_cpp' directory."; exit 1; }

vcs import "$inputPath" < "$inputPath"/libraries.repos || { echo "Failed to download dualsense library !"; exit 1; }
echo -e "\nLibraries downloaded.\n"
