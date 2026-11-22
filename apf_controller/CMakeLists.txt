cmake_minimum_required(VERSION 3.1)
project(apf_controller)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS roscpp nav_msgs geometry_msgs std_msgs tf)

catkin_package(CATKIN_DEPENDS roscpp nav_msgs geometry_msgs std_msgs tf)

add_executable(${PROJECT_NAME}_node src/apf_controller_node.cpp)
target_include_directories(${PROJECT_NAME}_node PRIVATE include ${catkin_INCLUDE_DIRS})
add_dependencies(${PROJECT_NAME}_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(${PROJECT_NAME}_node ${catkin_LIBRARIES})

install(TARGETS ${PROJECT_NAME}_node RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY config launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
