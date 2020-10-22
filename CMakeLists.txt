cmake_minimum_required(VERSION 2.8.3)
project(cslibs_mesh_map)

list(APPEND CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    roscpp
    tf
    visualization_msgs
    cslibs_math_3d
    cslibs_math_ros
    )

find_package(OpenMesh REQUIRED)
find_package(Eigen3 REQUIRED)

catkin_package(
    INCLUDE_DIRS include
    LIBRARIES cslibs_mesh_map
    CATKIN_DEPENDS roscpp tf visualization_msgs cslibs_math_3d cslibs_math_ros
    DEPENDS OpenMesh Eigen3
    )

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
    ${OPENMESH_INCLUDE_DIRS}
    ${Eigen3_INCLUDE_DIR}
    )

#cslibs_mesh_map_show_headers()

## Declare a C++ library
add_library(${PROJECT_NAME}
    src/cslibs_mesh_map_visualization.cpp
    src/edge_particle.cpp
    src/mesh_map_tree.cpp
    src/mesh_map.cpp
    )


add_dependencies(${PROJECT_NAME} ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Declare a C++ executable
## With catkin_make all packages are built within a single CMake context
## The recommended prefix ensures that target names across packages don't collide

## Add cmake target dependencies of the executable
## same as for the library above


## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
    ${OPENMESH_LIBRARIES}
    )

set(BUILD_MESHMAP_EXAMPLE True)
if(${BUILD_MESHMAP_EXAMPLE})
    message("====> Building mesh map example!")
    add_executable(${PROJECT_NAME}_example_node src/examples/cslibs_mesh_map_example_node.cpp)
    add_executable(${PROJECT_NAME}_file_tester src/examples/test_obj_file.cpp)
    add_dependencies(${PROJECT_NAME}_example_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    add_dependencies(${PROJECT_NAME}_file_tester ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
    target_link_libraries(${PROJECT_NAME}_example_node
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OPENMESH_LIBRARIES}
        )
    target_link_libraries(${PROJECT_NAME}_file_tester
        ${PROJECT_NAME}
        ${catkin_LIBRARIES}
        ${OPENMESH_LIBRARIES}
        )
endif()


#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

## Mark executable scripts (Python etc.) for installation
## in contrast to setup.py, you can choose the destination
# install(PROGRAMS
#   scripts/my_python_script
#   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark executables and/or libraries for installation
# install(TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )

## Mark cpp header files for installation
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
#   FILES_MATCHING PATTERN "*.h"
#   PATTERN ".svn" EXCLUDE
# )

## Mark other files for installation (e.g. launch and bag files, etc.)
# install(FILES
#   # myfile1
#   # myfile2
#   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
# )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_cslibs_mesh_map.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
