cmake_minimum_required(VERSION 2.8.3)
project(convex_solver)

add_definitions(-std=c++11)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  cmake_utils
)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_LIST_DIR}/cmake)

target_architecture(ARCH)

if(${ARCH} MATCHES "x86_64")
else()
    message("${Red}ARCH:${ARCH}$ do not suppoet mosek 8.0 abort to compile TOSP ${ColourReset}")
endif()

include_directories(
  include
)


if(${ARCH} MATCHES "x86_64")
    message("${Green}ARCH:${ARCH}${ColourReset}")
    catkin_package(
      INCLUDE_DIRS include
      LIBRARIES ${PROJECT_NAME} gp
      CATKIN_DEPENDS roscpp
      DEPENDS system_lib
    )

    file(GLOB ${PROJECT_NAME}_SRCS src/fusion_cxx/*.cc)
    add_library(${PROJECT_NAME}
        ${${PROJECT_NAME}_SRCS}
        )

    link_directories( ${CMAKE_CURRENT_LIST_DIR}/lib/mosek8 )

    target_link_libraries(${PROJECT_NAME}
        ${catkin_LIBRARIES}
        mosek64
        mosekxx8_1
        mosekxx8_0
        )

    file(GLOB GP_SRCS src/gp/*.cpp)
    add_library(gp
        ${GP_SRCS}
        )

    target_link_libraries(gp
        ${catkin_LIBRARIES}
        mosek64
        mosekxx8_1
        mosekxx8_0
        )


else()
    message("${Red}ARCH:${ARCH}$ do not suppoet mosek 8.0 abort to compile convex_solver ${ColourReset}")
endif()

