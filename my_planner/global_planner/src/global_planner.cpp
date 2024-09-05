#include "global_planner/global_planner.h"

namespace global_planner {


    inline void GlobalPlanner::initGridMap(GridMap::Ptr& map,
                                           const Eigen::Vector3i& pool_size) {
        pool_size_ = pool_size;
        grid_map_ = map;
        int allocated_node_num = pool_size_.sum();
        node_pool_.resize(allocated_node_num);
        for(int i = 0; i < allocated_node_num; i++) {
            node_pool_[i] = new Node();
        }

        grid_map_->getRegion(origin_, map_size_);
        resolution_ = grid_map_->getResolution();
        inv_resolution = 1.0 / resolution;

    } 

    inline Eigen::Vector3d GlobalPlanner::getMapsize() const {
        return this->map_size_;
    }

    inline void GlobalPlanner::setMapsize(const Eigen::Vector3d& map_size) {
        this->map_size_ = map_size;
    }

    inline std::vector<Eigen::Vector3d> GlobalPlanner::getPath() const {
        return this->path_;
    }

    inline Eigen::Vector3d GlobalPlanner::Coord2Index(const Eigen::Vector3d& coord) const {
        
    }

    inline Eigen::Vector3d GlobalPlanner::Index2Coord(const Eigen::Vector3d& index) const {
        
    }

    inline bool GlobalPlanner::isCollision(const Eigen::Vector3d& point) {
        // TODO: check if point is in collision
        return grid_map_->getInflateOccupancy(point);

    }

    inline void GlobalPlanner::isCollision(const std::vector<Eigen::Vector3d>& path) {
        // TODO: check if path is in collision
        for (auto& point : path) {
            if (isCollision(point)) {
                return true;
            }
        }

    }

}