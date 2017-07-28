## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  genmsg
  sensor_msgs
  visualization_msgs
  geometry_msgs
  message_generation
  dynamic_reconfigure
)

catkin_package(
    #  INCLUDE_DIRS include
    #  LIBRARIES sensor_fusion
       CATKIN_DEPENDS message_runtime 
    #  DEPENDS system_lib
)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations

## Declare a C++ library
# add_library(${PROJECT_NAME}
#   src/${PROJECT_NAME}/sensor_fusion.cpp
# )

include_directories(
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_SOURCE_DIR}/sensor_fusion/include
    )

 
add_executable(sensor_fusion_node FaceFuse.cpp)
target_link_libraries(sensor_fusion_node ${catkin_LIBRARIES})
add_dependencies(sensor_fusion_node sensor_fusion_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(sensor_fusion_node
   ${catkin_LIBRARIES}
)

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_sensor_fusion.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)


