cmake_minimum_required(VERSION 3.5)
project(gazebo_ros_four_wheel_steering)

if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(gazebo_dev REQUIRED)
find_package(gazebo_ros REQUIRED)
find_package(rclcpp REQUIRED)
find_package(four_wheel_steering_msgs REQUIRED)
find_package(control_msgs REQUIRED)

add_library(${PROJECT_NAME} SHARED src/plugin.cpp src/controller.cpp)
target_include_directories(${PROJECT_NAME} PUBLIC ./include)
target_include_directories(${PROJECT_NAME} PRIVATE ./src)
ament_target_dependencies(${PROJECT_NAME}
  "gazebo_dev"
  "gazebo_ros"
  "rclcpp"
  "four_wheel_steering_msgs"
  "control_msgs"
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)
  ament_lint_auto_find_test_dependencies()

  file(GLOB worlds RELATIVE ${CMAKE_CURRENT_SOURCE_DIR} "test/worlds/*.world")
  foreach(world ${worlds})
    configure_file(${world} ${world} COPYONLY)
  endforeach()

  ament_add_gtest(test_plugin
    test/test_plugin.cpp
    WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    TIMEOUT 5
  )
  target_link_libraries(test_plugin
    gazebo_test_fixture
    ${PROJECT_NAME}
  )
  ament_target_dependencies(test_plugin
    "gazebo_dev"
    "gazebo_ros"
    "rclcpp"
    "four_wheel_steering_msgs"
    "control_msgs"
  )
endif()

install(TARGETS ${PROJECT_NAME}
   DESTINATION lib
)

ament_export_libraries(${PROJECT_NAME})
ament_package()
