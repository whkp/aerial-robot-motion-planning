#pragma once

#include <queue>
#include <vector>
#include <iostream>
#include <ros/ros.h>
#include <ros/console.h>
#include <Eigen/Eigen>
#include <cmath>

#include "plan_env/grid_map.h"
#include "plan_env/global_planner.h"

namespace global_planner {
    
constexpr double inf = 1 << 30;

struct GridNode {
    enum node_state {
        IN_OPEN_LIST = 1,
        IN_CLOSE_LIST = 2,
        UN_VISITED = 3
    };

    enum node_state state;
    Eigen::Vector3d pos;
    GridNodePtr parent;
    double g_score;
    double f_score;

    GridNode() {
        state = UN_VISITED;
        parent = nullptr;
        g_score = inf;
        f_score = inf;
    }
};

class NodeHashTable {
  private:
    // don't use Eigen::Vector3i index as key, because different position may have the same index
    std::unordered_map<Eigen::Vector3d, PathNodePtr, hash_func<Eigen::Vector3d>> node_table_;
  
  public:
    NodeHashTable() {};
    ~NodeHashTable() {};

    void insert(Eigen::Vector3d pos, PathNodePtr node) {
      node_table_.insert(std::make_pair(pos, node));
    }

    PathNodePtr find(Eigen::Vector3d pos) {
      auto it = node_table_.find(pos);
      if (it != node_table_.end()) {  
        return it->second;
      } else {
        return NULL;
      }
    }

    void erase(Eigen::Vector3d pos) {
      node_table_.erase(pos);
    }

    void clear() {
      node_table_.clear();
    }


};

typedef std::shared_ptr<GridNode> GridNodePtr;

//node比较器，用于优先队列
class NodeComparator {
public:
    bool operator()(const GridNodePtr& a, const GridNodePtr& b) {
        return a->f_score > b->f_score;
    }
}

class AStar : public GlobalPlanner<GridNode>{
protected:
    std::priority_queue<GridNodePtr, std::vector<GridNodePtr>, NodeComparator> open_list_;
    NodeHashTable close_list_, expanded_list_;

    double tie_breaker; // 防止两个节点的f_score相等，导致无法确定优先级
    double lamda; // 调整启发式函数的权重
    Eigen::Vector3d goal_pos_;
    int use_node_num; // 记录已扩展的节点数

    vector<GridNodePtr> retrievePath(GridNodePtr current);
    void GetNeighbors(const GridNodePtr& current);
    double GetDiagHeu(const GridNodePtr start, const GridNodePtr goal);
    inline GetHeu(const GridNodePtr start, const GridNodePtr goal) {
        return tie_breaker * GetDiagHeu(start, goal);
    }

public:
    bool Plan(const Eigen::Vector3d& start, const Eigen::Vector3d& goal) override;
    void setParam(ros::NodeHandle& nh) override;
    void reset() override;

    typedef std::shared_ptr<AStar> AStarPtr;
    
    using GlobalPlanner::GlobalPlanner;
    ~AStar() ;
    
}

} // namespace global_planner

