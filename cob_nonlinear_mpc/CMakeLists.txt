cmake_minimum_required(VERSION 2.8.3)
project(cob_nonlinear_mpc)
add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS cmake_modules cob_control_msgs cob_srvs dynamic_reconfigure eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp roslint sensor_msgs std_msgs tf tf_conversions trajectory_msgs urdf visualization_msgs)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen REQUIRED)
add_definitions(${EIGEN_DEFINITIONS})

find_package(orocos_kdl REQUIRED)


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake/Modules/")

find_package(Ceres REQUIRED)
find_package(ACADO REQUIRED)
find_package(CASADI REQUIRED)
find_package(IPOPT REQUIRED)

message("-- Found Ceres version ${CERES_VERSION}: ${CERES_INCLUDE_DIRS}")

catkin_package(
  CATKIN_DEPENDS cob_control_msgs cob_srvs eigen_conversions geometry_msgs kdl_conversions kdl_parser nav_msgs pluginlib roscpp sensor_msgs std_msgs tf tf_conversions urdf visualization_msgs
  DEPENDS Boost CERES
  INCLUDE_DIRS include
  LIBRARIES
)

### BUILD ###
include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS} ${ACADO_INCLUDE_DIRS} )

set(SRC_C_DIR "src/constraint_solvers")
set(SRC_CS_DIR "${SRC_C_DIR}/solvers")

FIND_LIBRARY(libacado libacado_toolkit_s.so ${ACADO_DIR}/build/lib)

## internal libraries

add_library(nonlinear_mpc src/forward_kinematics.cpp src/bounding_volumes.cpp src/cob_nonlinear_mpc.cpp src/nonlinear_mpc.cpp src/utilities/nonlinear_mpc_utilities.cpp)
add_dependencies(nonlinear_mpc ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(nonlinear_mpc ${CASADI_LIBRARIES} ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

add_executable(cob_nonlinear_mpc_node src/cob_nonlinear_mpc_node.cpp)
add_dependencies(cob_nonlinear_mpc_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(cob_nonlinear_mpc_node nonlinear_mpc ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})


roslint_cpp()

### INSTALL ###
install(TARGETS cob_nonlinear_mpc_node
 ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
 RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/cob_nonlinear_mpc/
 DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)
