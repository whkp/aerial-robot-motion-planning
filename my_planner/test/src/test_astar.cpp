#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <plan_env/grid_map.h>
#include <global_planner/astar.h>

ros::Subscriber sub_goal;
ros::Subscriber sub_odom;

ros::Publisher pub_path;
nav::msgs::Odometry::ConstPtr odom;

std::vector<Eigen::Vector3d> path;

global_planner::AStar::AstarPtr astar_;

void OdomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    odom = msg;
}

void GoalCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    Eigen::Vector3d goal_pos(msg->pose.pose.position.x, msg->pose.pose.position.y, msg->pose.pose.position.z);
    Eigen::Vector3d start_pos(odom->pose.pose.position.x, odom->pose.pose.position.y, odom->pose.pose.position.z);
    ROS_INFO("Start Point:" << start_pos.transpose());
    ROS_INFO("Goal Point:" << goal_pos.transpose());
    bool success = global_planner::Astar::Plan(start_pos, goal_pos, path);
    if (success) {
        visualization_msgs::Marker path_marker;
        path_marker.header.frame_id = "world";
        path_marker.header.stamp = ros::Time::now();

        path_marker.ns = "astar/path";
        path_marker.id = 0;

        path_marker.type = visualization_msgs::Marker::LINE_STRIP;
        path_marker.action = visualization_msgs::Marker::ADD;

        path_marker.pose.orientation.w = 1.0;

        path_marker.scale.x = 0.1;
        path_marker.scale.y = 0.1;
        path_marker.scale.z = 0.1;

        path_marker.color.a = 1.0;
        path_marker.color.r = 1.0;
        path_marker.color.g = 0.0;
        path_marker.color.b = 0.0;

        for (int i = 0; i < path.size(); i++) {
            geometry_msgs::Point pt;
            pt.x = path[i][0];
            pt.y = path[i][1];
            pt.z = path[i][2];
            path_marker.points.push_back(pt);
        }

        path_pub.publish(path_marker);
    }
    else {
        ROS_INFO("No Path Found!");
    }
    path.clear();
    astar->reset();
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "test_astar");
    ros::NodeHandle nh("~");

    sub_goal = nh.subscribe<geometry_msgs::PoseStamped>("/goal", 1, GoalCallback);
    sub_odom = nh.subscribe<nav_msgs::Odometry>("/visual_slam/odom", 1, OdomCallback);

    pub_path = nh.advertise<visualization_msgs::Marker>("/astar/path", 1);
    GridMap::Ptr grid_map = std::make_shared<GridMap>();
    grid_map->initMap(nh);


    astar_ = std::make_shared<global_planner::Astar>();
    astar_->setParam(nh);
    astar_->initGridMap(grid_map, Eigen::Vector3i(100, 100, 100));
    

    ros::spin();
    return 0;
}