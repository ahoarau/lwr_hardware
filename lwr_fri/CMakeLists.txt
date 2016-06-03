cmake_minimum_required(VERSION 2.8.3)
project(lwr_fri)

find_package(catkin REQUIRED COMPONENTS
	rtt_roscomm
	kuka_lwr_fri 
	diagnostic_msgs 
	geometry_msgs 
	std_msgs 
	message_generation 
	kdl_conversions 
	cmake_modules
	)

add_message_files(DIRECTORY msg)
generate_messages(DEPENDENCIES geometry_msgs std_msgs)

catkin_package(
  DEPENDS rtt
  CATKIN_DEPENDS roscpp diagnostic_msgs geometry_msgs std_msgs message_runtime
)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
find_package(Eigen REQUIRED)
find_package(orocos_kdl REQUIRED)

orocos_use_package(rtt-transport-corba)
orocos_use_package(rtt-transport-mqueue)

ros_generate_rtt_typekit(lwr_fri)

if(${OROCOS_TARGET} STREQUAL "xenomai" )
  find_package(Xenomai REQUIRED)
  set_property(DIRECTORY ${PROJECT_SOURCE_DIR} APPEND PROPERTY COMPILE_DEFINITIONS HAVE_RTNET XENOMAI)
endif()

include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

orocos_component(fricomponent src/FRIComponent.cpp)
target_link_libraries(fricomponent ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES} ${Xenomai_RTDM_LIBRARIES})
add_dependencies(fricomponent ${PROJECT_NAME}_generate_messages_cpp)

orocos_component(fridiagnostics src/FRIDiagnostics.cpp)
target_link_libraries(fridiagnostics ${catkin_LIBRARIES})
add_dependencies(fridiagnostics ${PROJECT_NAME}_generate_messages_cpp)

orocos_generate_package(DEPENDS lwr_fri
          DEPENDS_TARGETS
          rtt_roscomm  
          rtt_std_msgs
          rtt_rosgraph_msgs
          rtt_geometry_msgs
          rtt_sensor_msgs)
