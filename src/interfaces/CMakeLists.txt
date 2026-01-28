cmake_minimum_required(VERSION 3.8)
project(interfaces)

# set(CMAKE_C_COMPILER clang)
# set(CMAKE_CXX_COMPILER clang++)
# set(CMAKE_EXE_LINKER_FLAGS"${CMAKE_EXE_LINKER_FLAGS} -stdlib=libc++ -lc++abi")
# cmake_policy(SET CMP0148 NEW)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Export compile commands for clangd
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
# 包含消息文件
rosidl_generate_interfaces(${PROJECT_NAME}
"message/ToolCenterPose.msg"
"service/GetTetris.srv"
"service/PutTetris.srv"
"service/PixelToBase.srv"
"service/ResetMotion.srv"

DEPENDENCIES
    std_msgs
    sensor_msgs
)    

ament_package()