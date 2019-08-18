cmake_minimum_required(VERSION 2.8.3)
project(sparrow_planner)

find_package(catkin REQUIRED
        COMPONENTS
            roscpp
            tf
            nav_core
            costmap_2d
            base_local_planner
            pluginlib
            genmsg
            actionlib_msgs
            nav_msgs
            rospy
            message_generation
        )

include_directories(
    include 
    ${catkin_INCLUDE_DIRS}
    )
add_definitions(${EIGEN3_DEFINITIONS})
add_service_files(
  FILES
  GetPlan.srv
  GetPathIdx.srv
)
add_action_files(DIRECTORY action FILES sparrow.action)
add_message_files(
    FILES
    grinding_path.msg
)
generate_messages(DEPENDENCIES actionlib_msgs geometry_msgs nav_msgs)
catkin_package(
    INCLUDE_DIRS include
    LIBRARIES sparrow_planner
    CATKIN_DEPENDS
        roscpp
        pluginlib
        costmap_2d
        base_local_planner
        nav_core
        message_runtime
)
add_library(sparrow_planner src/sparrow_planner.cpp)
add_dependencies(sparrow_planner ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(sparrow_planner
    ${catkin_LIBRARIES}
    )

install(TARGETS sparrow_planner
       ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
       )

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
  PATTERN ".svn" EXCLUDE
)

install(FILES bgp_plugin.xml
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

add_dependencies(sparrow_planner sparrow_planner_generate_messages_cpp)
