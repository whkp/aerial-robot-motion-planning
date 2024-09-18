cmake_minimum_required(VERSION 3.0.2)
project(traj_optimization)


find_package(catkin REQUIRED
    roscpp
    rospy
    std_msgs
    plan_env
    global_planner
    traj_optimization
    cv_bridge)


catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES traj_optimization
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)


