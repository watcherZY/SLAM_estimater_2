cmake_minimum_required(VERSION 2.8.3)
project(vi_slam)
######################
### Cmake flags
######################
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

find_package(catkin REQUIRED COMPONENTS
    roscpp
    std_msgs
    # msg
    geometry_msgs
    nav_msgs
    tf
    # cv
    cv_bridge
    )

find_package(OpenCV REQUIRED)

find_package(Ceres REQUIRED)
include_directories(${catkin_INCLUDE_DIRS} 
                    ${CERES_INCLUDE_DIRS})



set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake)
find_package(Eigen3)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${EIGEN3_INCLUDE_DIR}
)
catkin_package()


add_executable(${PROJECT_NAME}_estimator
    estimator/src/estimator_node.cpp
    estimator/src/parameters.cpp
    estimator/src/estimator.cpp
    estimator/src/backend.cpp
    estimator/src/initial.cpp
    estimator/src/feature_manager.cpp
    estimator/src/factor/pose_local_parameterization.cpp
    estimator/src/factor/projection_factor.cpp
    estimator/src/factor/projection_td_factor.cpp
    estimator/src/factor/marginalization_factor.cpp
    estimator/src/utility/utility.cpp
    estimator/src/utility/visualization.cpp
    estimator/src/utility/CameraPoseVisualization.cpp
    estimator/src/initial/solve_5pts.cpp
    estimator/src/initial/initial_aligment.cpp
    estimator/src/initial/initial_sfm.cpp
    estimator/src/initial/initial_ex_rotation.cpp
    )
target_link_libraries(${PROJECT_NAME}_estimator ${catkin_LIBRARIES} ${OpenCV_LIBS} ${CERES_LIBRARIES})
