cmake_minimum_required(VERSION 2.4.6)
project(monocular_pose_estimator_lib)

set(CMAKE_CXX_FLAGS "-O3  -std=c++11")

find_package(catkin REQUIRED COMPONENTS roscpp rospy cmake_modules)
find_package(Eigen REQUIRED)
find_package(OpenCV REQUIRED)


catkin_package(
  INCLUDE_DIRS include
  DEPENDS eigen opencv
  LIBRARIES ${PROJECT_NAME}
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${EIGEN_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

set(SOURCEFILES src/cliques.cpp src/combinations.cpp src/led_detector.cpp src/p3p.cpp src/pose_estimator.cpp src/stereo_pose_estimator.cpp src/visualization.cpp)

add_library( ${PROJECT_NAME} SHARED ${SOURCEFILES})
add_dependencies( ${PROJECT_NAME} SHARED ${catkin_EXPORTED_TARGETS} ) 
target_link_libraries( ${PROJECT_NAME} ${catkin_LIBRARIES} ${OpenCV_LIBS})

add_executable(cliques
  src/cliques.cpp
)
