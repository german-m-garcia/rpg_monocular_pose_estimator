cmake_minimum_required(VERSION 2.4.6)
project(monocular_pose_estimator)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS} -g -O3")

# search for everything we need to build the package
find_package(catkin REQUIRED COMPONENTS
  roscpp
  cv_bridge
  image_transport
  dynamic_reconfigure
  sensor_msgs
  geometry_msgs
  monocular_pose_estimator_lib
  GLRenderer
  cmake_modules
  nodelet
  tf_conversions  
  image_geometry
)

# since we need eigen search it as well
# find_package makes the ${..._INCLUDE_DIRS} ${..._LIBRARIES} variables we use later
find_package(Eigen REQUIRED)

# Add opencv
find_package(OpenCV REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLUT REQUIRED)

# Generate the reconfigure options
generate_dynamic_reconfigure_options(cfg/MonocularPoseEstimator.cfg)

# export the dependencies of this package for who ever depends on us
catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS roscpp cv_bridge image_transport dynamic_reconfigure sensor_msgs geometry_msgs monocular_pose_estimator_lib GLRenderer nodelet
  DEPENDS eigen OpenCV glut opengl
)

# tell catkin where to find the headers for this project
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

# make the node
add_executable(monocular_pose_estimator
  src/node.cpp src/monocular_pose_estimator.cpp
)
add_dependencies(monocular_pose_estimator ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)

target_link_libraries(monocular_pose_estimator
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OPENGL_LIBRARIES}
   ${GLUT_LIBRARY}
 )
 
add_executable(stereo_pose_estimator
  src/stereo_node.cpp src/stereo_pose_estimator.cpp
)
add_dependencies(stereo_pose_estimator ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)

target_link_libraries(stereo_pose_estimator
   yaml-cpp
   ${catkin_LIBRARIES}
   ${OpenCV_LIBRARIES}
   ${OPENGL_LIBRARIES}
   ${GLUT_LIBRARY}
 )
 

# make the nodelet
#add_library(monocular_pose_estimator_nodelet
#  src/nodelet.cpp src/monocular_pose_estimator.cpp
#)

#add_dependencies(monocular_pose_estimator_nodelet ${catkin_EXPORTED_TARGETS}  ${PROJECT_NAME}_gencfg)

#target_link_libraries(monocular_pose_estimator_nodelet
#   yaml-cpp
#  ${catkin_LIBRARIES}
#   ${OpenCV_LIBRARIES}
# )
 
