cmake_minimum_required(VERSION 3.5)
project(iras_behaviortree_ros2)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++20
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 20)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)

file (GLOB SOURCES_COMPONENTS src/components/*.cpp)
file (GLOB SOURCES_TOOLS src/tools/*.cpp)
file (GLOB SOURCES_CORE src/core/*.cpp)

add_library(behaviortree_ros_lib
  ${SOURCES_COMPONENTS}
  ${SOURCES_TOOLS}
  ${SOURCES_CORE}
)

include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${rclcpp_action_INCLUDE_DIRS}
)

target_link_libraries(behaviortree_ros_lib 
  PUBLIC ${rclcpp_LIBRARIES}
  PUBLIC ${rclcpp_action_LIBRARIES}
  PUBLIC BT::behaviortree_cpp_v3
)

# Install library
install(DIRECTORY include/
    DESTINATION include)

install(TARGETS behaviortree_ros_lib
    EXPORT export_behaviortree_ros_lib
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include)

ament_export_include_directories(include)
ament_export_targets(export_behaviortree_ros_lib HAS_LIBRARY_TARGET)
ament_export_dependencies(behaviortree_cpp_v3)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # uncomment the line when a copyright and license is not present in all source files
  #set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # uncomment the line when this package is not in a git repo
  #set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
