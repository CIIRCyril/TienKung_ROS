cmake_minimum_required(VERSION 3.5)
project(MasterStack)

add_subdirectory(soem)

find_package(Boost REQUIRED)

include_directories(${Boost_INCLUDE_DIR})
include_directories(
    soem/soem
    soem/osal
    soem/osal/linux
    soem/oshw/linux
)

set(CMAKE_CXX_STANDARD 17)

set(SOURCES 
    SoemMaster.cpp 
    SoemDevice.cpp
)
LINK_LIBRARIES(m)
add_library(soem_master SHARED ${SOURCES})
target_link_libraries(soem_master soem)

install(TARGETS soem soem_master
  LIBRARY DESTINATION lib
)