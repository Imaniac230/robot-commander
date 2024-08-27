cmake_minimum_required(VERSION 3.12)
project(library_vendor_cpp)

# find dependencies
find_package(ament_cmake REQUIRED)

add_subdirectory(input)

message(WARNING "This package does NOT currently install any external dependencies, please compile all external libraries manually using 'cmake' or use the 'library_vendor_manual_setup.sh' script.")

ament_package()
