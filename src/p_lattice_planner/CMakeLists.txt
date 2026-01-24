cmake_minimum_required(VERSION 3.8)
project(p_lattice_planner)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(nav2_core REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(nav2_util REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pinn REQUIRED)

# LibTorch
execute_process(
  COMMAND python3 -c "import torch; import os; print(os.path.join(os.path.dirname(torch.__file__), 'share/cmake/Torch'), end='')"
  OUTPUT_VARIABLE TORCH_CMAKE_PATH
)
if(TORCH_CMAKE_PATH)
  list(APPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PATH}")
endif()
find_package(Torch REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/lattice_planner.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${TORCH_INCLUDE_DIRS}
)

target_link_libraries(${PROJECT_NAME}
  ${TORCH_LIBRARIES}
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
  nav2_core
  nav2_costmap_2d
  nav2_util
  geometry_msgs
  nav_msgs
  pluginlib
  tf2_ros
  pinn
)

pluginlib_export_plugin_description_file(nav2_core plugin.xml)

install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include/
)

install(DIRECTORY launch maps scripts
  DESTINATION share/${PROJECT_NAME}
)

install(FILES plugin.xml benchmark_config.yaml nav2_params_snippet.yaml
  DESTINATION share/${PROJECT_NAME}
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)
ament_package()
