cmake_minimum_required(VERSION 3.5)

project(db-CBS)

find_package(Boost 1.58 REQUIRED COMPONENTS program_options)
find_package(Eigen3 REQUIRED)
find_package(PkgConfig)
pkg_check_modules(YAML REQUIRED yaml-cpp)
pkg_check_modules(FCL REQUIRED fcl ccd)



# set(ompl_ROOT ${}/k_cbs/ompl)
# find_package(ompl)
# pkg_check_modules(OMPL REQUIRED ompl)

# Enable C++17 and warnings
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_CXX_FLAGS  "${CMAKE_CXX_FLAGS} -Wall -Wextra")

add_subdirectory(k_cbs/Kinodynamic-Conflict-Based-Search EXCLUDE_FROM_ALL)

include_directories(
  ${EIGEN3_INCLUDE_DIR}
  ${CMAKE_SOURCE_DIR}/deps/libMultiRobotPlanning/include
  ${CMAKE_SOURCE_DIR}/k_cbs/Kinodynamic-Conflict-Based-Search/src
  ${CMAKE_BINARY_DIR}/k_cbs/Kinodynamic-Conflict-Based-Search/src
)

add_library(motion_planning_common
  src/robots.cpp
)
target_include_directories(motion_planning_common
  PRIVATE ${CMAKE_BINARY_DIR}/deps/fcl/include
  PRIVATE ${CMAKE_SOURCE_DIR}/deps/fcl/include
)
set_property(TARGET motion_planning_common PROPERTY POSITION_INDEPENDENT_CODE ON)

## ompl
add_executable(main_ompl
  src/main_ompl.cpp
  # src/robots.cpp
)
target_link_libraries(main_ompl
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

## kcbs

add_executable(main_kcbs
  src/main_kcbs.cpp
  # src/robots.cpp
)
target_link_libraries(main_kcbs
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_kcbs PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

## dbastar

add_executable(dbastar
  src/main_dbastar.cpp
  
)
target_link_libraries(dbastar
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(dbastar PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

## main_ompl_geometric

add_executable(main_ompl_geometric
  src/main_ompl_geometric.cpp
 
)
target_link_libraries(main_ompl_geometric
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)
target_include_directories(main_ompl_geometric PUBLIC 
  ${YAML_INCLUDE_DIRS}
  ${FCL_INCLUDE_DIRS}
)

# Python bindings
add_subdirectory(deps/pybind11 EXCLUDE_FROM_ALL)

pybind11_add_module(motionplanningutils
  src/python_bindings.cpp
)
target_include_directories(motionplanningutils
  PRIVATE ${CMAKE_BINARY_DIR}/deps/fcl/include
  PRIVATE ${CMAKE_SOURCE_DIR}/deps/fcl/include
)
target_link_libraries(motionplanningutils PRIVATE
  motion_planning_common
  ${Boost_LIBRARIES}
  ${YAML_LIBRARIES}
  ompl
  ${FCL_LIBRARIES}
)