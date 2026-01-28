cmake_minimum_required(VERSION 3.8)
project(camera)

SET(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_BUILD_TYPE Debug)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


######################
# Find dependencies ##
######################
add_compile_options(-g)

include_directories(${CMAKE_SOURCE_DIR}/include)
include_directories(${CMAKE_SOURCE_DIR}/lib/orbbecsdk/include)
include_directories(${CMAKE_SOURCE_DIR}/../../utils/check_utils)
link_directories(${CMAKE_SOURCE_DIR}/lib/orbbecsdk/lib)
set(OrbbecSDK_DIR lib/orbbecsdk/lib)

find_package(rclcpp_components REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_auto REQUIRED)
find_package(OrbbecSDK REQUIRED)
find_package(OpenCV REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# find_package(cv_bridge REQUIRED)
ament_auto_find_build_dependencies()
##########
# Build ##
##########

ament_auto_add_library(${PROJECT_NAME} SHARED
    DIRECTORY src
)
# target_compile_definitions(${PROJECT_NAME} PRIVATE
#     PKG_SOURCE_DIR=${CMAKE_CURRENT_SOURCE_DIR})
add_compile_definitions(PKG_SOURCE_DIR=\"${CMAKE_CURRENT_SOURCE_DIR}/\")
rclcpp_components_register_node(${PROJECT_NAME}
    PLUGIN CameraNode
    EXECUTABLE ${PROJECT_NAME}_node
)
ament_target_dependencies(${PROJECT_NAME}_node OrbbecSDK rclcpp OpenCV)
target_link_libraries(${PROJECT_NAME}_node ${OpenCV_LIBS})
target_link_libraries(${PROJECT_NAME} OrbbecSDK ${OpenCV_LIBS})
############
# Install ##
############
ament_auto_package(
    INSTALL_TO_SHARE
)
