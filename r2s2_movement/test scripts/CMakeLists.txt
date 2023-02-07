cmake_minimum_required(VERSION 3.0.2)
project(r2s2_calibration)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED
    COMPONENTS
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    message_generation
    )

add_message_files(
	FILES
	calibration_msg.msg
	)

generate_messages(
	DEPENDENCIES
	std_msgs
	)

## PACKAGE ##
catkin_package(
	CATKIN_DEPENDS message_runtime
	)

## Install
install(DIRECTORY launch scripts msg
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
