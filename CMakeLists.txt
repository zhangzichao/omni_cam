project(omni_cam)
cmake_minimum_required (VERSION 2.8.3)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

set(HEADERS include/omni_cam/ocam.h)

set(SOURCES src/ocam.cpp)

cs_add_library(${PROJECT_NAME} ${SOURCES} ${HEADERS})

################################################################################
# Tests

catkin_add_gtest(${PROJECT_NAME}_tests
  test/test_ocam.cpp
)
target_link_libraries(${PROJECT_NAME}_tests ${PROJECT_NAME})

################################################################################
# Install

cs_install()
cs_export()
