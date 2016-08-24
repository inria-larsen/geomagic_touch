# Copyright: (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
# Authors: Ugo Pattacini
# CopyPolicy: Released under the terms of the GNU GPL v2.0.

cmake_minimum_required(VERSION 2.8.9)
project(test_geomagic_yarp)

find_package(YARP         REQUIRED)
find_package(ICUBcontrib  REQUIRED)
find_package(hapticdevice REQUIRED)
find_package(gazebo REQUIRED)

list(APPEND CMAKE_MODULE_PATH ${YARP_MODULE_PATH})
list(APPEND CMAKE_MODULE_PATH ${ICUBCONTRIB_MODULE_PATH})

include(ICUBcontribOptions)
include(ICUBcontribHelpers)
icubcontrib_set_default_prefix()


include_directories(${YARP_INCLUDE_DIRS} ${hapticdevice_INCLUDE_DIRS})

add_library(fake_geomagic_driver fakeGeomagicDriver.h fakeGeomagicDriver.cpp)
target_link_libraries(fake_geomagic_driver ${YARP_LIBRARIES}  )
    
add_definitions(-D_USE_MATH_DEFINES)
add_executable(test_simple test_simple.cpp)
target_link_libraries(test_simple ${YARP_LIBRARIES})
install(TARGETS test_simple DESTINATION bin)

add_executable(test_orientation test_orientation.cpp)
target_link_libraries(test_orientation ${YARP_LIBRARIES})
install(TARGETS test_orientation DESTINATION bin)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")
add_executable(test_gazebo test_gazebo.cpp cartesianClient.cpp)
target_link_libraries(test_gazebo ${YARP_LIBRARIES} ${GAZEBO_LIBRARIES} pthread fake_geomagic_driver)

add_executable(test_feedback test_feedback.cpp cartesianClient.cpp)
target_link_libraries(test_feedback ${YARP_LIBRARIES} pthread fake_geomagic_driver)

install(TARGETS test_simple DESTINATION bin)
install(TARGETS test_gazebo DESTINATION bin)
install(TARGETS test_feedback DESTINATION bin)
install(TARGETS test_orientation DESTINATION bin)
