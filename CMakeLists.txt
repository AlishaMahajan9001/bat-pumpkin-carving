cmake_minimum_required(VERSION 3.8)
project(cpp_path_planner)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(pumpkin_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Declare Executable
add_executable(client src/moon_path_planner.cpp)

# Include Directory
target_include_directories(client PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# Link Libraries
target_link_libraries(client
                      yaml-cpp
                      )

# Add Dependencies
ament_target_dependencies(client 
            rclcpp 
            pumpkin_msgs 
            geometry_msgs
            trajectory_msgs
            visualization_msgs
            )

# Declare Executable with Dependencies
add_executable(path_plot_subscriber src/path_plot_subscriber.cpp)
ament_target_dependencies(path_plot_subscriber rclcpp geometry_msgs)

# Install
install(
  TARGETS 
  client
  path_plot_subscriber
  DESTINATION lib/${PROJECT_NAME})

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_package()
