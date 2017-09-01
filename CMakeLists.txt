cmake_minimum_required(VERSION 2.8.3)
project(dynamixel_tilt)

find_package(catkin REQUIRED COMPONENTS
     tf
     tf_conversions
     roscpp
     dynamixel_msgs
     laser_assembler
     laser_geometry
    )
include_directories(
    include
    ${Boost_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS}
    )
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES ${PROJECT_NAME}
    CATKIN_DEPENDS tf std_msgs roscpp
     DEPENDS Boost
)
#include_directories(  ${DYNAMIXEL_TILT_SOURCE_DIR_DIRS}/include)

## Declare a cpp executable
add_executable(motor_tilt_node src/motor_tilt_node.cpp)
target_link_libraries(motor_tilt_node ${catkin_LIBRARIES})

add_executable(call_assemble src/call_assemble_scans.cpp)
target_link_libraries(call_assemble ${catkin_LIBRARIES})

add_executable(tfScan2Base src/tfLaserScan2BaseTilt.cpp)
target_link_libraries(tfScan2Base ${catkin_LIBRARIES})
install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
