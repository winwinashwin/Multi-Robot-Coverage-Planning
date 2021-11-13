cmake_minimum_required(VERSION 3.2)
project(topics_rviz_plugin)
add_compile_options(-Wall -Wextra)
set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  class_loader
  roscpp
  rqt_gui
  rviz
)

## Find Qt5
set(CMAKE_AUTOMOC ON)
find_package(Qt5Widgets REQUIRED)

set(
  PROJECT_QT_PANELS_TARGETS
  ${PROJECT_NAME}_display_topics
)

set(
  PROJECT_TARGETS
  ${PROJECT_QT_PANELS_TARGETS}
  ${PROJECT_NAME}_topic_info
)

################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
  INCLUDE_DIRS
  include
  LIBRARIES
  ${PROJECT_TARGETS}
  CATKIN_DEPENDS
  roscpp
  rqt_gui
  rviz
)

###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

# TopicInfo class / library
add_library(
  ${PROJECT_NAME}_topic_info
  include/${PROJECT_NAME}/topic_info.hpp
  src/topic_info.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_topic_info
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_topic_info
  ${catkin_EXPORTED_TARGETS}
)

# Qt panel - Display topics
add_library(
  ${PROJECT_NAME}_display_topics
  include/${PROJECT_NAME}/display_topics.hpp
  src/display_topics.cpp
)
target_link_libraries(
  ${PROJECT_NAME}_display_topics
  ${PROJECT_NAME}_topic_info
  Qt5::Widgets
  ${catkin_LIBRARIES}
)
add_dependencies(
  ${PROJECT_NAME}_display_topics
  ${catkin_EXPORTED_TARGETS}
)

# Needed by Qt / RViz / ROS
class_loader_hide_library_symbols(
  ${PROJECT_QT_PANELS_TARGETS}
)

#############
## Install ##
#############

## Mark executables and/or libraries for installation
# Qt panel
install(TARGETS
  ${PROJECT_TARGETS}
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

## Mark cpp header files for installation
install(
  DIRECTORY
  include/${PROJECT_NAME}/
  DESTINATION
  ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
)

## Mark other files for installation (e.g. launch and bag files, etc.)
install(
  FILES
  plugin.xml
  DESTINATION
  ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
