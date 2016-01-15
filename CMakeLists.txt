cmake_minimum_required(VERSION 2.8)
project(can_driver)

find_package(catkin REQUIRED COMPONENTS
    roscpp cmake_modules
  )

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES can_driver
)

find_package(OROCOS-RTT REQUIRED)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  include_directories(include ${catkin_INCLUDE_DIRS} ${Xenomai_RTDM_INCLUDE_DIRS})
  add_definitions(-DHAVE_RTNET)
  add_definitions(-DXENOMAI)
else()
  include_directories(include ${catkin_INCLUDE_DIRS})
endif()

add_library(can_driver src/CANDev.cpp)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  target_link_libraries(can_driver ${catkin_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
else()
  target_link_libraries(can_driver ${catkin_LIBRARIES})
endif()

