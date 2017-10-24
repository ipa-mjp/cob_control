cmake_minimum_required(VERSION 2.8.3)
project(mpc_controller)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp roslint sensor_msgs std_msgs tf tf_conversions trajectory_msgs urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

SET(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")
SET(ACADO_DIR /home/bfb-ws/ACADOtoolkit/)
SET(ACADO_INCLUDE_PACKAGES ${ACADO_DIR} ${ACADO_DIR}/acado ${ACADO_DIR}/external_packages)

find_package(orocos_kdl REQUIRED)

find_package(Ceres REQUIRED)
find_package(ACADO REQUIRED)

message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs
  DEPENDS Boost CERES ACADO
  INCLUDE_DIRS include ${ACADO_INCLUDE_PACKAGES}
  LIBRARIES
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${ACADO_INCLUDE_PACKAGES} )
FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

add_executable(kinematics_test test/kinematics_test.cpp src/kinematics.cpp)

add_dependencies(kinematics_test ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

target_link_libraries(kinematics_test  
	${catkin_LIBRARIES} 
	${orocos_kdl_LIBRARIES}
	${libacado} 
)

