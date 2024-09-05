#pragma once
//planner based on grid map

#include <unordered_map>
#include <vector>
#include <iostream>
#include <Eigen/Eigen>
#include <ros/ros.h>

#include "plan_env/grid_map.h"

namespace global_planner {

template<typename Node>
class GlobalPlanner {
    //using NodePtr = Node*;
    using NodePtr = std::shared_ptr<Node>;
public:    
    GlobalPlanner() = default;

    virtual ~GlobalPlanner() = default;

    virtual bool plan(const Eigen::Vector3d& start, 
                      const Eigen::Vector3d& goal,
                      std::vector<Eigen::Vector3d>& path) = 0;
    
    virtual void setParam();

    virtual void reset();

    inline void initGridMap(GridMap::Ptr& map, const Eigen::Vector3i& pool_size);

    inline bool isCollision(const Eigen::Vector3d& point);

    inline bool isCollision(const std::vector<Eigen::Vector3d>& points);

    inline Eigen::Vector3d getMapsize() const;
    
    inline void setMapsize(const Eigen::Vector3d& map_size);

    inline std::vector<Eigen::Vector3d> getPath() const;

    inline Eigen::Vector3d Coord2Index();
    
    inline Eigen::Vector3d Index2Coord();

protected:
    // main map parameters
    Eigen::Vector3d origin_;
    Eigen::Vector3d map_size_;

    Eigen::Vector3i pool_size_;
    std::vector<Eigen::Vector3d> path_;
    double resultion_;
    double inv_resolution_;
    GridMap::Ptr grid_map_;
    std::vector<NodePtr> node_pool_;
};
}