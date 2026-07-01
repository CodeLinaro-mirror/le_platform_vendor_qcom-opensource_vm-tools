project(vm_powerctl)

cmake_minimum_required(VERSION 3.10)

add_definitions(-D_GNU_SOURCE)

set(SOURCE_FILES
    vm_powerctl.c
)

add_executable(vm_powerctl ${SOURCE_FILES})

include_directories(${SYSROOTINC_PATH})
include_directories(${SYSROOT_INCLUDEDIR})

link_directories(${SYSROOT_LIBDIR})
link_directories(${CMAKE_INSTALL_LIBDIR})

target_link_libraries(vm_powerctl vmm_client)

install(TARGETS vm_powerctl DESTINATION bin)
