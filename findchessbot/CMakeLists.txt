cmake_minimum_required(VERSION 3.5)
project(findchessbot)

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


################ BUILD DEPENDENCIES  #################
find_package(ament_cmake REQUIRED)          # c++    (optional)
find_package(rclcpp REQUIRED)               # c++    (optional)
find_package(ament_cmake_python REQUIRED)   # python (optional)
find_package(rclpy REQUIRED)                # python (optional)
######################################################
################ FIND DEPENDENCIES  ##################
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
######################################################
include_directories(include)
######################################################


find_package(rosidl_default_generators REQUIRED)
set(srv_files 
  "srv/Capture.srv"
  "srv/Castling.srv"
  "srv/MoveChesspiece.srv"
  "srv/PicknPlace.srv"
  "srv/PoseGripper.srv"
  "srv/PoseJoint.srv"
  "srv/PromotePawn.srv"
  "srv/McuMode.srv"
)
set(msg_files 
  "msg/JogJoint.msg"
  "msg/JogLinear.msg"
)
set(action_files 
  "action/PickandPlace.action"
)
rosidl_generate_interfaces(${PROJECT_NAME}
  ${srv_files}
  ${msg_files}
  ${action_files}
  DEPENDENCIES std_msgs geometry_msgs
  ADD_LINTER_TESTS
)

################### INSTALL C++  #####################
# add_library(cpp_lib src/cpp_lib.cpp)
# add_executable(cpp_node src/cpp_node.cpp)
# target_link_libraries(cpp_node rclcpp::rclcpp cpp_lib)

# install(TARGETS cpp_node
#   DESTINATION lib/${PROJECT_NAME})
####################### END ##########################

################## INSTALL Python  ###################
ament_python_install_package(python_lib)
# Install Python executables
install(PROGRAMS
  # scripts/python_node.py
  scripts/Server.py
  # scripts/input_chessboard_rpm.py  
  scripts/Robot_jointstate.py  
  scripts/Chessboard_jointstate.py
  # scripts/input_to_jointstate.py   
  scripts/Server.py       
  scripts/Super_OS.py
  scripts/Action_call.py
  scripts/Action_Server.py
  scripts/Test_traj.py
  scripts/Serial_Commu.py
  DESTINATION lib/${PROJECT_NAME}
)

####################### END ##########################
################ INSTALL LAUNCH, ETC #################
install(DIRECTORY
  launch
  config
  urdf
  config
  meshes
  rviz
  DESTINATION share/${PROJECT_NAME})
####################### END ##########################
ament_export_dependencies(rosidl_default_runtime)
ament_package() # always required
