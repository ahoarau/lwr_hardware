cmake_minimum_required(VERSION 2.8.3)
project(lwr_fri)
set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/config)

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
  CATKIN_DEPENDS roscpp diagnostic_msgs geometry_msgs std_msgs message_runtime
)

find_package(OROCOS-RTT REQUIRED ${RTT_HINTS})
find_package(Eigen3 REQUIRED)
find_package(orocos_kdl REQUIRED)

orocos_use_package(rtt-transport-corba)
orocos_use_package(rtt-transport-mqueue)

ros_generate_rtt_typekit(lwr_fri)

orocos_component(fricomponent src/FRIComponent.cpp)

find_package(Xenomai QUIET)
# Allows to build for Xenomai + RTnet
if(XENOMAI_FOUND OR (DEFINED $ENV{OROCOS_TARGET} AND $ENV{OROCOS_TARGET} EQUAL "xenomai"))

    message("[${PROJECT_NAME}] Building lwr_fri for Xenomai ${XENOMAI_VERSION} + RTnet")
    message("[${PROJECT_NAME}] Compiling with ${XENOMAI_RTDM_LIBRARIES} and ${XENOMAI_RTDM_INCLUDE_DIRS} and ${XENOMAI_RTDM_CFLAGS}")

    # In Xenomai 3 it contains RTnet
    include_directories(${XENOMAI_RTDM_INCLUDE_DIRS})
    add_definitions(${XENOMAI_RTDM_CFLAGS})

    # In Xenomai 2 we need the external RTnet
    if(${XENOMAI_VERSION_MAJOR} EQUAL 2)
        find_package(RTnet REQUIRED)
        include_directories(${RTNET_INCLUDE_DIRS})
    endif()

    set_target_properties(fricomponent PROPERTIES COMPILE_FLAGS -DXENOMAI_VERSION_MAJOR=${XENOMAI_VERSION_MAJOR})
else()
    message(WARNING "[${PROJECT_NAME}] Building lwr_fri WITHOUT Xenomai/RTnet support")
endif()


include_directories(include ${catkin_INCLUDE_DIRS} ${Eigen_INCLUDE_DIRS})
link_directories(${orocos_kdl_LIBRARY_DIRS})

target_link_libraries(fricomponent ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES} ${kdl_conversions_LIBRARIES} ${XENOMAI_RTDM_LIBRARIES})
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
