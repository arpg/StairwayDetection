cmake_minimum_required(VERSION 2.8 FATAL_ERROR)

project(stair_detection)

#SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++2a")

find_package(PCL 1.8 REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(catkin REQUIRED
  COMPONENTS
    roscpp
    sensor_msgs
    visualization_msgs
    #pcl_ros
    #message_generation
    tf_conversions
    tf
    dynamic_reconfigure
)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choosing Release build type - the fastest option."
       FORCE )
endif()

generate_dynamic_reconfigure_options(
  cfg/StairDetection.cfg
)

#add_message_files(
#  DIRECTORY
#    msg
#  FILES
#    PointCloud2Array.msg
#  )

#generate_messages(
#  DEPENDENCIES
#    sensor_msgs
#)

catkin_package(
  CATKIN_DEPENDS roscpp 
  )

include_directories(
    ${catkin_INCLUDE_DIRS}
    #${colormap_SOURCE_DIR}/include
    )

include_directories (${PCL_INCLUDE_DIRS})
include_directories ("${PROJECT_SOURCE_DIR}/include")

link_directories (${PCL_LIBRARY_DIRS})
add_definitions (${PCL_DEFINITIONS})

add_executable (stair_det 
  src/stairDetectionMain.cpp  
  src/preanalysis.cpp 
  src/regions.cpp 
  src/segmentPatch.cpp 
  src/regiongrowing.cpp 
  src/voxSAC.cpp 
  src/splitmerge.cpp 
  src/planeshape.cpp 
  src/Stairs.cpp 
  src/StairVector.cpp 
  src/recognition.cpp
  )
target_link_libraries (stair_det 
  ${PCL_LIBRARIES}
  yaml-cpp
  )

#set  (project_SOURCES src/stairDetectionMain.cpp   src/regiongrowing.cpp src/preanalysis.cpp src/regiongrowing-orig.cpp src/regions.cpp src/segmentPatch.cpp src/clusteranalysis.cpp src/splitmerge.cpp src/recognition.cpp src/planeshape.cpp src/StairRegistration.cpp src/StairCoefficients.cpp src/Stairs.cpp src/StairVector.cpp src/referenceSegmentation.cpp src/voxelSAC.cpp src/RANSAC.cpp src/splitNormalEstimation.cpp)

add_executable( stair_detection_node
  src/stair_detection_node.cpp
  src/preanalysis.cpp 
  src/regions.cpp 
  src/segmentPatch.cpp 
  src/regiongrowing.cpp 
  src/voxSAC.cpp 
  src/splitmerge.cpp 
  src/planeshape.cpp 
  src/Stairs.cpp 
  src/StairVector.cpp 
  src/recognition.cpp
  #src/prediction.cpp
)
add_dependencies( stair_detection_node
  stair_detection_gencfg
  stair_detection_gencpp
)
target_link_libraries( stair_detection_node
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)

add_executable( example_stairs_publisher
  src/example_stairs_publisher.cpp
)
target_link_libraries( example_stairs_publisher
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
)
