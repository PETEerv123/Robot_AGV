cmake_minimum_required(VERSION 3.8)
project(robot_lidar)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)

# 1. Đường dẫn SDK
set(RPLIDAR_SDK "${CMAKE_CURRENT_SOURCE_DIR}/../rplidar_ros/sdk")

# 2. CHỈ LẤY FILE CỦA LINUX (Bỏ qua macOS và Win32)
file(GLOB RPLIDAR_SDK_SRC "${RPLIDAR_SDK}/src/*.cpp")
file(GLOB RPLIDAR_HAL_SRC "${RPLIDAR_SDK}/src/hal/*.cpp")
file(GLOB RPLIDAR_ARCH_SRC "${RPLIDAR_SDK}/src/arch/linux/*.cpp")
file(GLOB_RECURSE RPLIDAR_UNPACKER_SRC "${RPLIDAR_SDK}/src/dataunpacker/*.cpp")

set(RPLIDAR_SOURCES
    ${RPLIDAR_SDK_SRC}
    ${RPLIDAR_HAL_SRC}
    ${RPLIDAR_ARCH_SRC}
    ${RPLIDAR_UNPACKER_SRC}
)

# 3. Include các thư mục Header
include_directories(
    ${RPLIDAR_SDK}/include
    ${RPLIDAR_SDK}/src
    ${RPLIDAR_SDK}/src/hal
    ${RPLIDAR_SDK}/src/arch/linux
    ${RPLIDAR_SDK}/src/dataunpacker
    ${RPLIDAR_SDK}/src/dataunpacker/unpacker
)

# 4. Tạo file thực thi
add_executable(robot_lidar
    src/rplidar_serial.cpp
    ${RPLIDAR_SOURCES}
)

ament_target_dependencies(robot_lidar
    rclcpp
    sensor_msgs
)

target_link_libraries(robot_lidar
    pthread
)

install(TARGETS robot_lidar
    DESTINATION lib/${PROJECT_NAME}
)

ament_package()
