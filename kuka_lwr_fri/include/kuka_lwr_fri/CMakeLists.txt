find_package(Orocos-RTT REQUIRED ${RTT_HINTS})
# Use custom version of orocos_typegen_headers waiting for PR
include(${CMAKE_CURRENT_SOURCE_DIR}/UseOROCOS-RTT.cmake)
#include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake)
orocos_use_package(orogen)
orocos_typegen_headers(${CMAKE_CURRENT_SOURCE_DIR}/friComm.h)
