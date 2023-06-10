cmake_minimum_required(VERSION 3.8)
project(usv_package)

set(CMAKE_BUILD_TYPE Debug)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(OpenMP REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(PNG REQUIRED)
find_package(nav_msgs REQUIRED)

include_directories(./include)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

aux_source_directory(${PROJECT_SOURCE_DIR}/src DIR_MAIN_SRCS)
add_executable(usv_main ${DIR_MAIN_SRCS})
ament_target_dependencies(usv_main rclcpp std_msgs sensor_msgs geometry_msgs tf2_msgs tf2_geometry_msgs nav_msgs PCL)

target_link_libraries(usv_main OpenMP::OpenMP_CXX)
target_link_libraries(usv_main ${PCL_LIBRARIES})

install(TARGETS 
    usv_main 
    DESTINATION lib/${PROJECT_NAME})


ament_package()
