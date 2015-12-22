find_package(Orocos-RTT REQUIRED ${RTT_HINTS})
# We need to install those headers : catkin install; catkin build
# or just make install in the build directory of kuka_lwr_fri (not recommended)
include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
orocos_use_package(orogen)
orocos_typegen_headers(${CMAKE_CURRENT_SOURCE_DIR}/friComm.h)
