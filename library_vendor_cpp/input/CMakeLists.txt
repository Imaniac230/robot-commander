cmake_minimum_required(VERSION 3.5)
project(input)

# Default to C99
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 99)
endif ()

# Default to C++17
if (NOT CMAKE_CXX_STANDARD)
    set(CMAKE_CXX_STANDARD 20)
endif ()

if (CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif ()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_srvs REQUIRED)
#NOTE(sdl2-vendored): Adding a local subdirectory with sdl2 here doesn't work with colcon builds (works only with
# standard cmake building). When using colcon, the compiled input library seems to always link to the system sdl2
# and not the newer imported one even if it was actually compiled together. If we want to use a newer sdl2 version with
# ROS, we must override the sdl2_vendor package (download the joystick_drivers ROS package and build its sdl2_vendor locally with FORCE_BUILD_VENDOR_PKG=ON).
find_package(SDL2 REQUIRED)
find_package(robot_commander_interfaces REQUIRED)

set(CORE_LIB_NAME ${PROJECT_NAME})
set(GAMEPAD_NODE_EXEC_NAME gamepad_node)
set(ROS_DEPENDENCIES rclcpp std_msgs geometry_msgs std_srvs robot_commander_interfaces)


include_directories(PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
        ${SDL2_INCLUDE_DIRS})

if ("${SDL2_LIBRARIES}" STREQUAL "")
    message(WARNING "SDL2_LIBRARIES wasn't set, manually setting to SDL2::SDL2")
    set(SDL2_LIBRARIES "SDL2::SDL2")
endif ()

add_subdirectory(lib)

add_library(${CORE_LIB_NAME} SHARED
        src/DualsenseCtlWrapper.cpp
        src/GamepadBase.cpp
        src/GamepadNode.cpp
)
ament_target_dependencies(${CORE_LIB_NAME} ${ROS_DEPENDENCIES})
target_link_libraries(${CORE_LIB_NAME} ${SDL2_LIBRARIES})
#FIXME(dualsensectl-library) we cannot currently add the dualsensectl project directly as a library since it's created
#   as a single main cli application, until an actual usable library is found we have a simple wrapper for the cli commands instead
#add_dependencies(${CORE_LIB_NAME} dualsensectl)

add_executable(${GAMEPAD_NODE_EXEC_NAME} src/gamepad_node.cpp)
target_link_libraries(${GAMEPAD_NODE_EXEC_NAME} ${CORE_LIB_NAME})

install(TARGETS ${GAMEPAD_NODE_EXEC_NAME}
        DESTINATION lib/${PROJECT_NAME})

install(
        TARGETS
        ${CORE_LIB_NAME}
        ARCHIVE DESTINATION lib
        LIBRARY DESTINATION lib
        RUNTIME DESTINATION bin)

# must export include if this library is used in other projects
install(DIRECTORY include/ DESTINATION include/)

if (BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
endif ()

ament_export_include_directories(include)
ament_export_libraries(${CORE_LIB_NAME})
ament_package()
