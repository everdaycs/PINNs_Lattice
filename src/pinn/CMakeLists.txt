cmake_minimum_required(VERSION 3.8)
project(pinn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
# We might not need rclcpp if this is pure C++, but usually logging uses it.
# Check if FeasibilityModel uses rclcpp logging. It used std::iostream.
# So rclcpp might not be needed.

# LibTorch
execute_process(
  COMMAND python3 -c "import torch; import os; print(os.path.join(os.path.dirname(torch.__file__), 'share/cmake/Torch'), end='')"
  OUTPUT_VARIABLE TORCH_CMAKE_PATH
)
if(TORCH_CMAKE_PATH)
  list(APPEND CMAKE_PREFIX_PATH "${TORCH_CMAKE_PATH}")
endif()

set(USE_CUDA OFF)
find_package(Torch REQUIRED)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

# JSON
find_package(nlohmann_json REQUIRED)

include_directories(include)

# Create Library
add_library(${PROJECT_NAME} SHARED
  src/FeasibilityModel.cpp
  src/EdgeFeatureExtractor.cpp
  src/SpeedCorrectorModel.cpp
  src/EdgeDynamicsEvaluator.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} 
  "${TORCH_LIBRARIES}"
  nlohmann_json::nlohmann_json
)

ament_target_dependencies(${PROJECT_NAME}
  rclcpp
  rclcpp_lifecycle
)

# Install
install(TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(nlohmann_json)
# Note: Torch is usually not an ament package, so we don't ament_export_dependencies(Torch) 
# usually. Downstream packages need `find_package(Torch)` themselves or we use cmake extras.
# For simplicity, we assume downstream does find_package(Torch).

# Install environment hook for libtorch path
install(FILES env-hooks/pinn_env.sh DESTINATION share/${PROJECT_NAME}/environment)
ament_environment_hooks("env-hooks/pinn_env.sh")

ament_package()
