cmake_minimum_required(VERSION 3.8)
project(map_changer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_lint_auto REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(PCL REQUIRED COMPONENTS common io filters)


if(BUILD_TESTING)
set(ament_cmake_copyright_FOUND TRUE)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()


# ノードのコンパイル
add_executable(map_change_node src/map_changer.cpp)


ament_target_dependencies(map_change_node
  std_msgs
  std_srvs
  nav2_msgs
  rclcpp
  sensor_msgs
  geometry_msgs
  pcl_conversions
)

target_link_libraries(map_change_node
  ${PCL_LIBRARIES}
)

# インストール
install(TARGETS
  map_change_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

endif()

ament_package()
