#include "global_planner/astar.h"

using namespace std;
using namespace Eigen;

namespace global_planner {
    void Astar::setParam(ros::NodeHandle &nh) override {
        nh.param("astar/tie_breaker", tie_breaker_, 1.0 + 1e-4);
        nh.param("astar/lambda_heu", lambda_, 1.0);
    }

    Astar::~Astar() {
        for(int i = 0; i < allocated_node_num; i++) {
            delete node_pool_[i];
        }
    }

    double Astar::GetDiagHeu(const Eigen::Vector3d x1, const Eigen::Vector3d x2) {
        double dx=std::abs(x1(0)- x2(0));
        double dy=std::abs(x1(1)- x2(1));
        double dz=std::abs(x1(2)- x2(2));

        double h = 0.0;
        int diag = min(min(dx, dy), dz);
        dx -= diag;
        dy -= diag;
        dz -= diag;

        if (dx == 0) {
            h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dy, dz) + 1.0 * abs(dy - dz);
        }
        if (dy == 0) {
            h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dz) + 1.0 * abs(dx - dz);
        }
        if (dz == 0) {
            h = 1.0 * sqrt(3.0) * diag + sqrt(2.0) * min(dx, dy) + 1.0 * abs(dx - dy);
        }
        return h;
    }

    void AStar::retrievePath(GridNodePtr current) {
        while (current -> parent != nullptr) {
            path_.push_back(current.pos);
            current = current -> parent;
        }
        path_.push_back(current -> pos);
        std::reverse(path_.begin(), path_.end());
    }

    void Astar::getNeighbors(const GridNodePtr& current) {
        if(current == nullptr) {
            ROS_ERROR("current node is nullptr");
            return;
        }
        for(double x = -resolution_; x <= resolution_; x += resolution_) {
            for(double y = -resolution_; y <= resolution_; y += resolution_) {
                for(double z = -resolution_; z <= resolution_; z += resolution_) {
                    Eigen::Vector3d neighbor_pos = current->pos + Eigen::Vector3d(x, y, z);
                    if(!grid_map_->isInMap(neighbor_pos) ) {
                        continue;
                    }
                    if(isCollision(neighbor_pos)) {
                        continue;
                    }
                    if(close_list_.find(neighbor_pos)) {
                        continue;
                    }

                    double tmp_g_score = current->g_score + sqrt(x*x + y*y + z*z);
                    GridNodePtr neighbor = close_list_.find(neighbor_pos);
                    if(neighbor == NULL) {
                        PathNodePtr neighbor_node = node_pool_[use_node_num];
                        use_node_num += 1;
                        neighbor_node->pos = neighbor_pos;
                        neighbor_node->g_score = tmp_g_score;
                        neighbor_node->f_score = tmp_g_score + lambda_*GetDiagHeu(neighbor_pos, goal_pos_);
                        neighbor_node->parent = current;
                        neighbor_node->state = GridNode::IN_OPEN_LIST;
                        open_list_.push(neighbor_node);
                        expanded_list_.insert(neighbor_pos, neighbor_node);
                        if(use_node_num >= allocated_node_num) {
                            ROS_ERROR("node pool is too small");
                            return;
                        }
                        else if(tmp_g_score < neighbor_node->g_score) {
                            neighbor_node->g_score = tmp_g_score;
                            neighbor_node->f_score = tmp_g_score + lambda_*GetDiagHeu(neighbor_pos, goal_pos_);
                            neighbor_node->parent = current;
                    }
                }
            }
        }
    }                    
        
    }

    bool Astar::plan(const Eigen::Vector3d& start_pos, 
                     const Eigen::Vector3d& goal_pos,
                     std::vector<Eigen::Vector3d>& path) override {
        ros::Time start_time = ros::Time::now();
        use_node_num = 0;
        goal_pos_ = goal_pos;
        if(!grid_map_->isInMap(goal_pos)) {
            ROS_ERROR("goal position is out of map");
            return false;
        }
        GridNodePtr start_node = node_pool_[use_node_num];
        start_node->g_score = 0;
        start_node->pos = start_pos;
        start_node->f_score = lambda_*GetDiagHeu(start_pos, goal_pos);
        start_node->parent = nullptr;
        start_node->state = GridNode::IN_OPEN_LIST;
        open_list_.push(start_node);

        use_node_num += 1;

        while(!open_list_.empty()) {
            GridNodePtr current = open_list_.top();
            open_list_.pop();
            current->state = GridNode::IN_CLOSE_LIST;
            close_list_.insert(current->pos, current);
            if(current->pos(0) - goal_pos(0) < resoulution_ &&
               current->pos(1) - goal_pos(1) < resoulution_ &&
               current->pos(2) - goal_pos(2) < resoulution_) {
                retrievePath(current);
                path = path_;
                ROS_INFO("reached goal !!!");
                ROS_INFO("use node num: %d", use_node_num);
                ROS_INFO("planning time: %f", (ros::Time::now() - start_time).toSec());
                return true;
            }
            getNeighbors(current);
            ros::Time time2 = ros::Time::now();
            if((time2 - start_time).toSec() > 0.5) {
                ROS_WARN("planning time exceeds 0.5s");
                return false;
            }

        }
    }

    void Astar::reset() override {
        use_node_num = 0;
        open_list_.clear();
        close_list_.clear();
        expanded_list_.clear();
        path_.clear();
        for(auto node: node_pool_) {
            node->state = GridNode::UN_VISITED;
            node->parent = nullptr;
            node->g_score = inf;
            node->f_score = inf;
        }
    }
}

