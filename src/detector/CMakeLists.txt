cmake_minimum_required(VERSION 3.8)
project(detector)


set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_CXX_EXTENSIONS ON)
set(CMAKE_CXX_STANDARD 20)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(OpenCV REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

set(ONNXRUNTIME_DIR "/opt/onnxruntime")
set(ONNXRUNTIME_LIB ${ONNXRUNTIME_DIR}/lib/libonnxruntime.so)
include_directories(${ONNXRUNTIME_DIR}/include)
link_directories(${ONNXRUNTIME_DIR}/lib)

ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

add_compile_definitions(PKG_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${ONNXRUNTIME_LIB}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_DIR}/include
)

rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN DetectorNode
    EXECUTABLE detector_node
)
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(
    INSTALL_TO_SHARE
)
