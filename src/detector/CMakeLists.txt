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

set(RKNNRT_DIR ${CMAKE_SOURCE_DIR}/lib/librknn_api/aarch64)
set(RKNNRT_INC_DIR ${CMAKE_SOURCE_DIR}/lib/librknn_api/include)
set(RKNNRT_LIB ${RKNNRT_DIR}/librknnrt.so)
include_directories(${RKNNRT_INC_DIR})

set(RGA_DIR ${CMAKE_SOURCE_DIR}/lib/rga/RK3588/lib/Linux/aarch64)
set(RGA_INC_DIR ${CMAKE_SOURCE_DIR}/lib/rga/RK3588/include)
set(RGA_LIB ${RGA_DIR}/librga.so)
include_directories(${RGA_INC_DIR})

include_directories(${CMAKE_SOURCE_DIR}/../utils)
include_directories(${CMAKE_SOURCE_DIR}/include/${PROJECT_NAME})


ament_auto_add_library(${PROJECT_NAME} SHARED
  DIRECTORY src
)

# --- 将属性设置移到这里 ---
set_target_properties(${PROJECT_NAME} PROPERTIES
  INSTALL_RPATH "$ORIGIN"
  BUILD_RPATH "${RKNNRT_DIR}"
)

add_compile_definitions(PKG_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")

target_link_libraries(${PROJECT_NAME}
  ${OpenCV_LIBRARIES}
  ${ONNXRUNTIME_LIB}
  ${RKNNRT_LIB}
  ${RGA_LIB}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${ONNXRUNTIME_DIR}/include
  ${RKNNRT_INC_DIR}
  ${RGA_INC_DIR}
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


install(FILES ${RKNNRT_LIB}
  DESTINATION lib/${PROJECT_NAME}
)

install(FILES ${RKNNRT_LIB}
  DESTINATION lib
)

ament_auto_package(
    INSTALL_TO_SHARE
)
