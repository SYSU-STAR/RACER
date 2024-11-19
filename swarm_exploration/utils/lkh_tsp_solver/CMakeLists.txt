cmake_minimum_required(VERSION 2.8.3)
project(lkh_tsp_solver)

set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_FLAGS "-O3 -Wall")

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
)

add_service_files(
  FILES
  SolveTSP.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
)

catkin_package(
 INCLUDE_DIRS include
 LIBRARIES lkh_tsp_solver
 CATKIN_DEPENDS roscpp
#  DEPENDS system_lib
)

include_directories( 
    SYSTEM 
    include 
    ${catkin_INCLUDE_DIRS} ${PROJECT_SOURCE_DIR}/include
)

file( GLOB SRCS src/*.c )

# message("${SRCS}")

add_library( lkh_tsp_solver 
  ${SRCS}
  src/lkh_interface.cpp
)
target_link_libraries(lkh_tsp_solver -lm)

# add_executable( tsp_node 
#   ${SRCS}
#   src2/tsp_node.cpp
# )
# target_link_libraries(tsp_node ${catkin_LIBRARIES} -lm lkh_tsp_solver)