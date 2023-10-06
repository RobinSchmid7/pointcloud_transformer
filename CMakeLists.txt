cmake_minimum_required(VERSION 2.8.3)
project(pointcloud_transformer)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        tf
        nodelet
        tf2_sensor_msgs
        )

catkin_package(
        #  INCLUDE_DIRS include
        LIBRARIES pointcloudTransformer
        #  CATKIN_DEPENDS roscpp sensor_msgs tf
        #  DEPENDS system_lib
)

###########
## Build ##
###########

include_directories(
        include
        ${catkin_INCLUDE_DIRS}
)

## Declare a C++ library
add_library(pointcloudTransformer
        src/pointcloud_transformer_nodelet.cpp
        )
add_dependencies(pointcloudTransformer ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(pointcloudTransformer
        ${catkin_LIBRARIES}
        )


install(TARGETS pointcloudTransformer
        DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION})
install(DIRECTORY include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION})

install(DIRECTORY plugins
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})

install(DIRECTORY launch
        DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
