cmake_minimum_required(VERSION 2.8.3)
project(ric_interface)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED roscpp)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ric_interface
#  CATKIN_DEPENDS other_catkin_pkg
 # DEPENDS system_lib
)

include_directories(
 include
 ${catkin_INCLUDE_DIRS}
)

set(
        PROJECT_FILES
        src/ric_interface.cpp
        src/serial_com.cpp
        src/timer.cpp
        include/${PROJECT_NAME}/ric_interface.h
        include/${PROJECT_NAME}/communicator.h
        include/${PROJECT_NAME}/serial_com.h
        include/${PROJECT_NAME}/protocol.h
        include/${PROJECT_NAME}/ric_exception.h
        include/${PROJECT_NAME}/timer.h
        include/${PROJECT_NAME}/sensors_state.h
        include/${PROJECT_NAME}/crc8.h
        src/ric_interface_node.cpp
)

## Declare a C++ library
add_library(
        ${PROJECT_NAME}
        ${PROJECT_FILES}
        )

## Add cmake target dependencies of the library
## as an example, code may need to be generated before libraries
## either from message generation or dynamic reconfigure
add_dependencies(
        ${PROJECT_NAME}
        ${${PROJECT_NAME}_EXPORTED_TARGETS}
        ${catkin_EXPORTED_TARGETS}
)

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide
add_executable(
        ${PROJECT_NAME}_node
        ${PROJECT_FILES}
        )

## Add cmake target dependencies of the executable
## same as for the library above
add_dependencies(${PROJECT_NAME}_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
   ${catkin_LIBRARIES}
        )

## Mark executables and/or libraries for installation
 install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark cpp header files for installation
 install(DIRECTORY include/${PROJECT_NAME}/
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h"
   PATTERN ".svn" EXCLUDE
 )