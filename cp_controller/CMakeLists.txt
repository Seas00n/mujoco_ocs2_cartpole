cmake_minimum_required(VERSION 3.0.2)
project(cp_controller)

# Generate compile_commands.json for clang tools
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CATKIN_PACKAGE_DEPENDENCIES
  ocs2_core
  ocs2_ddp
  ocs2_mpc
  ocs2_ros_interfaces
  ocs2_robotic_tools
  cp_interface
  ocs2_robotic_assets
)

find_package(catkin REQUIRED COMPONENTS
  roslib
  ${CATKIN_PACKAGE_DEPENDENCIES}
)

find_package(Boost REQUIRED COMPONENTS
  system
  filesystem
)

find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(glfw3 3.3 REQUIRED)
##########
##Mujoco##
##########
## change to your mujoco path
set(MUJOCO_INSTALL_INCLUDE_DIR $ENV{MUJOCO_INSTALL_INCLUDE})
set(MUJOCO_INSTALL_BIN_DIR $ENV{MUJOCO_INSTALL_BIN})
set(MUJOCO_INSTALL_LIB_DIR $ENV{MUJOCO_INSTALL_LIB})
if(NOT EXISTS ${MUJOCO_INSTALL_INCLUDE_DIR})
  message(FATAL_ERROR "\"MUJOCO_INSTALL_INCLUDE_DIR\" is invalid path: ${MUJOCO_INSTALL_INCLUDE_DIR}")
endif()
if(NOT EXISTS ${MUJOCO_INSTALL_BIN_DIR})
  message(FATAL_ERROR "\"MUJOCO_INSTALL_BIN_DIR\" is invalid path: ${MUJOCO_INSTALL_BIN_DIR}")
endif()
if(NOT EXISTS ${MUJOCO_INSTALL_LIB_DIR})
  message(FATAL_ERROR "\"MUJOCO_INSTALL_LIB_DIR\" is invalid path: ${MUJOCO_INSTALL_LIB_DIR}")
endif()
file(GLOB LIB_MUJOCO ${MUJOCO_INSTALL_LIB_DIR}/libmujoco[0-9][0-9][0-9].so
     ${MUJOCO_INSTALL_LIB_DIR}/libmujoco.so.*)
if(EXISTS ${LIB_MUJOCO})
  message(STATUS "MuJoCo lib found: " ${LIB_MUJOCO})
else()
  message(FATAL_ERROR "MuJoCo lib not found.")
endif()
###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
    include
    ${EIGEN3_INCLUDE_DIRS}
  CATKIN_DEPENDS
    ${CATKIN_PACKAGE_DEPENDENCIES}
  DEPENDS
    Boost
)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)
# Mpc node
add_executable(cartpole_mpc
  src/CartpoleMpcNode.cpp
)
add_dependencies(cartpole_mpc
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cartpole_mpc
  ${catkin_LIBRARIES}
)

# Dummy node
add_executable(cartpole_dummy_test
  src/DummyCartpoleNode.cpp
  src/CartpoleDummyVisualization.cpp
)
add_dependencies(cartpole_dummy_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cartpole_dummy_test
  ${catkin_LIBRARIES}
)

# Mujoco node 
add_executable(cartpole_mj_test
  src/CartpoleMujocoNode.cpp
  src/CartpoleDummyVisualization.cpp
)
target_include_directories(cartpole_mj_test
  PRIVATE
  ${MUJOCO_INSTALL_INCLUDE_DIR}
)
add_dependencies(cartpole_mj_test
  ${catkin_EXPORTED_TARGETS}
)
target_link_libraries(cartpole_mj_test
  ${catkin_LIBRARIES}
  ${LIB_MUJOCO}
  glfw
)



# #########################
# ###   CLANG TOOLING   ###
# #########################
# find_package(cmake_clang_tools QUIET)
# if(cmake_clang_tools_FOUND)
#   message(STATUS "Running clang tooling.")
#   add_clang_tooling(
#     TARGETS
#         cartpole_mpc
#         cartpole_dummy_test
#         cartpole_mj_test
#     SOURCE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/src ${CMAKE_CURRENT_SOURCE_DIR}/include
#     CT_HEADER_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include
#     CT_HEADER_EXCLUDE_DIRS ${CMAKE_CURRENT_SOURCE_DIR}/include/${PROJECT_NAME}/generated
#     CF_WERROR
#   )
# endif(cmake_clang_tools_FOUND)

# #############
# ## Install ##
# #############

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
# install(TARGETS cartpole_mpc cartpole_dummy_test
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY launch rviz
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )
