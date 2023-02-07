cmake_minimum_required(VERSION 3.0.2)
project(r2s2_movement)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
        moveit_core
        moveit_visual_tools
        moveit_ros_planning
        moveit_ros_planning_interface
        pcl_ros
        pcl_conversions
        rospy
        )

catkin_package(
    CATKIN_DEPENDS moveit_core moveit_visual_tools moveit_ros_planning_interface
    )


## Build
# include_directories()


## Install
install(DIRECTORY launch meshes scripts
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
