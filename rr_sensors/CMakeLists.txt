cmake_minimum_required(VERSION 2.8.3)
project(rr_sensors)

##################
## Dependencies ##
##################
add_compile_options(-std=c++11)

set(PKG_DEPS
    roscpp
    std_msgs
    rr_hw_interface
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

set(LSOC_LIBRARIES "-lsoc")

############
## catkin ##
############

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
    INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_executable(${PROJECT_NAME}
    src/analog_sensor.cpp
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${LSOC_LIBRARIES}
)

add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME}
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(FILES README.md
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
