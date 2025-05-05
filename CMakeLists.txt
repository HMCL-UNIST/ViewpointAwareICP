cmake_minimum_required(VERSION 3.10)
project(vicp)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

find_package(PCL REQUIRED)
find_package(Eigen3 REQUIRED)
# find_package(Ceres REQUIRED)
find_package(Open3D QUIET)
find_package(Boost REQUIRED COMPONENTS system)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIRS})

add_definitions(${PCL_DEFINITIONS})
include_directories(include)

add_executable(demo 
  src/demo.cpp 
  src/viewpointAwareICP.cpp 
  src/v_score.cpp
)
target_include_directories(demo PRIVATE include  ${PCL_INCLUDE_DIRS} )
target_link_libraries(demo ${PCL_LIBRARIES})

if (Open3D_FOUND)
  message(STATUS "✅ Found Open3D: ${Open3D_VERSION}")
  add_executable(demo_vis 
    src/demo_vis.cpp 
    src/viewpointAwareICP.cpp 
    src/v_score.cpp
  )

  target_include_directories(demo_vis PRIVATE ${Open3D_INCLUDE_DIRS})
  target_link_libraries(demo_vis
    ${Open3D_LIBRARIES}
    ${PCL_LIBRARIES}
    Boost::system
  )
endif()

