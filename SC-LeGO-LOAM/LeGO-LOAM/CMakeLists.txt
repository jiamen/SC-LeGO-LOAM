
cmake_minimum_required( VERSION 2.8.3 )
project( lego_loam )

# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14 -O3")
set( CMAKE_CXX_STANDARD 14 )

find_package( catkin REQUIRED COMPONENTS
        tf
        roscpp
        cv_bridge
        image_transport

        pcl_ros             # 连接ROS和pcl库
        pcl_conversions

        std_msgs
        sensor_msgs         # 定义了激光雷达的msg
        geometry_msgs
        nav_msgs
        cloud_msgs
)

find_package( GTSAM REQUIRED QUIET )
find_package( PCL REQUIRED QUIET )
find_package( OpenCV REQUIRED QUIET )

catkin_package(
        INCLUDE_DIRS include
        CATKIN_DEPENDS cloud_msgs
        DEPENDS PCL
)

include_directories(
        include
        ${catkin_INCLUDE_DIRS}      # 这个开始没添加，utility.h中所有的ros库无法找到
        ${OpenCV_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${GTSAM_INCLUDE_DIR}
)

add_executable(imageProjection src/imageProjection.cpp)
add_dependencies(imageProjection ${catkin_EXPORTED_TARGETS} cloud_msgs_gencpp)
target_link_libraries(imageProjection ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${OpenCV_LIBRARIES})



#add_executable(mapOptmization
#        ${src/mapOptization.cpp}
#)

