cmake_minimum_required(VERSION 2.8.3)
project(rr_hw_interface)

##################
## Dependencies ##
##################
add_compile_options(-std=c++11)

set(PKG_DEPS
  roscpp
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

set(LSOC_LIBRARIES "-lsoc")
set(REVPI_LIBRARIES "-lrevpi")

############
## catkin ##
############

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include
)

###########
## Build ##
###########

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/gpio/libsoc_gpio.cpp
  src/gpio/revpi_gpio.cpp
  )

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${LSOC_LIBRARIES}
    ${REVPI_LIBRARIES}
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(FILES README.md
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
