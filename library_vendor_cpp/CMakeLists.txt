cmake_minimum_required(VERSION 3.12)
project(library_vendor_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)

message(WARNING "This package currently does nothing, please compile all libraries manually using 'cmake' or use the 'library_vendor_manual_setup.sh' script.")

ament_package()
