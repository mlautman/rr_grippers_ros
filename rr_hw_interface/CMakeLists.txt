cmake_minimum_required(VERSION 2.8.3)
project(rr_hw_interface)

##################
## Dependencies ##
##################
#add_compile_options(-std=c++11)

set(PKG_DEPS
  roscpp
)

find_package(catkin REQUIRED COMPONENTS ${PKG_DEPS})

set(LSOC_LIBRARIES "-lsoc")
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

############
## catkin ##
############

catkin_package(
    CATKIN_DEPENDS ${PKG_DEPS}
    LIBRARIES ${PROJECT_NAME}
    INCLUDE_DIRS include external/librevpi/include external/mcp23017_libsoc
)

####################
## Subdirectories ##
####################
add_subdirectory(external/librevpi)
add_subdirectory(external/mcp23017_libsoc)

###########
## Build ##
###########
set(CMAKE_CXX_FLAGS "-std=c++11 -fPIC ${CMAKE_CXX_FLAGS}")

include_directories(
    include
    external/librevpi/include
    external/mcp23017_libsoc
    ${catkin_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} 
  src/gpio/libsoc_gpio.cpp
  src/gpio/revpi_gpio.cpp
  src/gpio/mcp23017_gpio.cpp
  src/aio/revpi_aio.cpp
)

add_dependencies(${PROJECT_NAME} 
  ${${PROJECT_NAME}_EXPORTED_TARGETS} 
  ${catkin_EXPORTED_TARGETS}
  librevpi
  mcp23017
)

target_include_directories(${PROJECT_NAME} PUBLIC
  include 
  external/librevpi/include
  external/mcp23017_libsoc
)

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${LSOC_LIBRARIES}
    ${CATKIN_DEVEL_PREFIX}/lib/liblibrevpi.a
    ${CATKIN_DEVEL_PREFIX}/lib/libmcp23017.a
)

#############
## Install ##
#############

install(TARGETS ${PROJECT_NAME}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

install(DIRECTORY include/${PROJECT_NAME} 
   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
   FILES_MATCHING PATTERN "*.h" PATTERN "*.hpp"
)

install(FILES README.md
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
