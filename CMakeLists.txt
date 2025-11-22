cmake_minimum_required(VERSION 3.8)
project(web_test)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)   

add_executable(web_server_node src/web_server_node.cpp)
target_include_directories(web_server_node PUBLIC include)
target_link_libraries(web_server_node pthread)
ament_target_dependencies(web_server_node rclcpp std_msgs ament_index_cpp)

add_executable(map_viewer_node src/map_viewer_node.cpp)
target_include_directories(map_viewer_node PUBLIC include)
target_link_libraries(map_viewer_node pthread)
ament_target_dependencies(map_viewer_node rclcpp nav_msgs geometry_msgs sensor_msgs tf2_ros tf2_geometry_msgs ament_index_cpp)

install(TARGETS web_server_node map_viewer_node
  DESTINATION lib/${PROJECT_NAME})

# Install config, launch, and web directories
install(DIRECTORY config launch web
  DESTINATION share/${PROJECT_NAME}/
  USE_SOURCE_PERMISSIONS)

# Install .env file separately
install(FILES .env
  DESTINATION share/${PROJECT_NAME}/)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
