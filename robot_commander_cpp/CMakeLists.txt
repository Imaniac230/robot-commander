cmake_minimum_required(VERSION 3.5)
project(robot_commander_cpp VERSION 0.0.1)

# Default to C11
if (NOT CMAKE_C_STANDARD)
    set(CMAKE_C_STANDARD 11)
endif ()

# Default to C++20
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
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(robot_commander_interfaces REQUIRED)
find_package(nlohmann_json REQUIRED)

set(CORE_LIB_NAME ${PROJECT_NAME})
set(CONTEXT_NODE_EXEC_NAME message_context_node)
set(ROS_DEPENDENCIES rclcpp std_msgs geometry_msgs nav_msgs std_srvs robot_commander_interfaces)


include_directories(PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>)

add_library(${CORE_LIB_NAME} STATIC
        src/MessageContextNode.cpp
)
ament_target_dependencies(${CORE_LIB_NAME} ${ROS_DEPENDENCIES})
target_link_libraries(${CORE_LIB_NAME} nlohmann_json)

add_executable(${CONTEXT_NODE_EXEC_NAME} src/message_context_node.cpp)
target_link_libraries(${CONTEXT_NODE_EXEC_NAME} ${CORE_LIB_NAME})

install(TARGETS ${CONTEXT_NODE_EXEC_NAME}
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
    find_package(ament_cmake_gtest REQUIRED)
    # the following line skips the linter which checks for copyrights
    # uncomment the line when a copyright and license is not present in all source files
    #set(ament_cmake_copyright_FOUND TRUE)
    # the following line skips cpplint (only works in a git repo)
    # uncomment the line when this package is not in a git repo
    #set(ament_cmake_cpplint_FOUND TRUE)
    ament_lint_auto_find_test_dependencies()
    enable_testing()

    add_subdirectory(test)
endif ()

ament_export_include_directories(include)
ament_export_libraries(${CORE_LIB_NAME})
ament_package()
