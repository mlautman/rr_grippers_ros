cmake_minimum_required(VERSION 2.8.3)
project(rr_suction_pump)

##################
## Dependencies ##
##################
add_compile_options(-std=c++11)

set(PKG_DEPS
    roscpp
    actionlib
    actionlib_msgs
    rr_hw_interface
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

#####################
## Code generation ##
#####################

add_action_files(
  DIRECTORY action
  FILES SuctionPump.action
)
generate_messages(
  DEPENDENCIES actionlib_msgs std_msgs
)

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

#gpio - gpio
add_executable(${PROJECT_NAME}_action_server_trigger_gpio_status_gpio
    src/suction_pump_action_server_trigger_gpio_status_gpio.cpp
)

target_link_libraries(${PROJECT_NAME}_action_server_trigger_gpio_status_gpio
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_action_server_trigger_gpio_status_gpio ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

#gpio - aio
add_executable(${PROJECT_NAME}_action_server_trigger_gpio_status_aio
    src/suction_pump_action_server_trigger_gpio_status_aio.cpp
)

target_link_libraries(${PROJECT_NAME}_action_server_trigger_gpio_status_aio
    ${catkin_LIBRARIES}
)

add_dependencies(${PROJECT_NAME}_action_server_trigger_gpio_status_aio ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})


#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}_action_server_trigger_gpio_status_gpio
    ${PROJECT_NAME}_action_server_trigger_gpio_status_aio
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
