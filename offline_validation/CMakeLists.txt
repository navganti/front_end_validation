cmake_minimum_required(VERSION 2.8.3)
project(offline_validation)

## gnu++11 feature
set(CMAKE_CXX_STANDARD 11)
if(NOT CMAKE_BUILD_TYPE)
    MESSAGE(STATUS "Using 'Release' build type as CMAKE_BUILD_TYPE is not set")
    set(CMAKE_BUILD_TYPE Release)
endif()

# Dependencies
find_package(Eigen3 REQUIRED)
find_package(Boost REQUIRED)
find_package(OpenCV REQUIRED)
find_package(wave REQUIRED containers vision utils)
find_package(yaml-cpp REQUIRED)

# Catkin dependencies
set(catkin_deps
  cv_bridge
  roscpp
  roslib
  sensor_msgs
  std_msgs
)
find_package(catkin REQUIRED ${catkin_deps} message_generation)

# Messages
add_message_files()
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
    INCLUDE_DIRS include
    CATKIN_DEPENDS ${catkin_deps}
    LIBRARIES ${PROJECT_NAME}
)

###########
## Build ##
###########

include_directories(SYSTEM
    ${catkin_INCLUDE_DIRS}
    ${Boost_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
)
include_directories(include)

# Declare source code
add_library(${PROJECT_NAME}
    src/offline_validator.cpp)

target_link_libraries(${PROJECT_NAME}
    wave::vision
    wave::containers
    wave::utils
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(${PROJECT_NAME}_node
    src/${PROJECT_NAME}_node.cpp
)
target_link_libraries(${PROJECT_NAME}_node
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############
install(TARGETS ${PROJECT_NAME}_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  FILES_MATCHING PATTERN "*.hpp"
)

install(DIRECTORY launch
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
