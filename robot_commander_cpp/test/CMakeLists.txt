find_package(GTest REQUIRED)

set(TEST_LIBS ${CORE_LIB_NAME} GTest::GTest GTest::Main)
set(TEST_DEPS ${ROS_DEPENDENCIES})

ament_add_gtest(messageContextTests messageContextTests.cpp WORKING_DIRECTORY ${CMAKE_CURRENT_LIST_DIR} APPEND_LIBRARY_DIRS ${TEST_LIBS})
ament_target_dependencies(messageContextTests ${TEST_DEPS})
