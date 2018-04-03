cmake_minimum_required(VERSION 2.8.3)
project(image_tracking)

add_definitions(-std=c++11)

#--------------------------#
# Generation of SDF models #
#--------------------------#

# We need Gazebo version >= 3.0.0 to generate iris.sdf file
# (gz sdf ... command needs to be available)
find_package(gazebo REQUIRED)

if(${gazebo_VERSION_MAJOR} GREATER 2)
    message(STATUS "Building iris.sdf.")
    
    set(enable_mavlink_interface "true")
    set(enable_ground_truth "false")
    set(enable_logging "false")
    set(enable_camera "false")
    set(enable_wind "false")
    set(rotors_description_dir "${CMAKE_CURRENT_SOURCE_DIR}/models/rotors_description")
    set(scripts_dir "${CMAKE_CURRENT_SOURCE_DIR}/scripts")
    
    # Creates shell commands to generate .sdf file
    add_custom_command(OUTPUT ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
      COMMAND rm -f ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND python ${scripts_dir}/xacro.py -o  ${rotors_description_dir}/urdf/iris_base.urdf  ${rotors_description_dir}/urdf/iris_base.xacro enable_mavlink_interface:=${enable_mavlink_interface} enable_ground_truth:=${enable_ground_truth} enable_wind:=${enable_wind} enable_logging:=${enable_logging} rotors_description_dir:=${rotors_description_dir}
      COMMAND gz sdf -p  ${rotors_description_dir}/urdf/iris_base.urdf >> ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf
      COMMAND rm -f ${rotors_description_dir}/urdf/iris_base.urdf
      DEPENDS ${rotors_description_dir}/urdf/iris.xacro
      DEPENDS ${rotors_description_dir}/urdf/iris_base.xacro
      DEPENDS ${rotors_description_dir}/urdf/component_snippets.xacro
      )
    add_custom_target(sdf ALL DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/models/iris/iris.sdf)
else()
  message(STATUS "Gazebo version is less than 3, cannot generate iris.sdf (gz tool not available).")
endif()

if(NO_ROS)
  message(STATUS "NO_ROS = true, not building waypoint publisher or hovering example, returning now.")
  return()
endif()


find_package(catkin REQUIRED COMPONENTS gazebo_msgs geometry_msgs octomap_ros mav_msgs roscpp sensor_msgs octomap_msgs message_generation)

add_service_files(
    FILES
	CastQuery.srv 
	Debug.srv 

  )
add_message_files(
  FILES
  ProposalBox.msg
  ProposalBoxes.msg
  ProposalBoxPath.msg
  ProposalRay.msg
  ProposalRays.msg

)
generate_messages(DEPENDENCIES geometry_msgs  )

catkin_package(
  INCLUDE_DIRS include
  CATKIN_DEPENDS  
    gazebo_msgs
    geometry_msgs
    mav_msgs
    roscpp
    sensor_msgs
    octomap_ros
    octomap_msgs
	message_runtime

)

#OPENCV
find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_libraries(${OpenCV_LIBRARIES})

include_directories(include ${catkin_INCLUDE_DIRS} ${CMAKE_CURRENT_SOURCE_DIR}/src ~/libigl/include)


add_executable(waypoint_publisher_from_key src/waypoint_publisher_from_key.cpp)
target_link_libraries(waypoint_publisher_from_key ${catkin_LIBRARIES})
add_dependencies(waypoint_publisher_from_key ${catkin_EXPORTED_TARGETS})


add_executable(waypoint_proposal_from_octomap_node src/waypoint_proposal_node_.cpp src/waypoint_proposal_.cpp src/optimization_funcs.cpp)
target_link_libraries(waypoint_proposal_from_octomap_node ${catkin_LIBRARIES} nlopt ${OpenCV_LIBRARIES} splinter-static-3-0)
add_dependencies(waypoint_proposal_from_octomap_node  ${catkin_EXPORTED_TARGETS})

#OCTOMAP 
find_package(octomap REQUIRED)
include_directories(${OCTOMAP_INCLUDE_DIRS})
link_libraries(${OCTOMAP_LIBRARIES})

