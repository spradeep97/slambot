#ifndef PLANNING_NODE_HPP
#define PLANNING_NODE_HPP

#include <lcmtypes/pose_xyt_t.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <common/point.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <vector>

class Node
{
public:
    Node(void);
    Node(const pose_xyt_t& pose, const ObstacleDistanceGrid& grid);
    void set_costs(double g, double h);
    bool operator == (const Node& node) const {
      if (node.pose_.x == pose_.x && node.pose_.y == pose_.y) {
        return true;
      }
      return false;
    }
    bool operator != (const Node& node);
    bool operator < (const Node& node) const {
      return (f_ < node.f_);
    }
    bool operator <= (const Node& node);
    bool operator > (const Node& node);
    bool operator >= (const Node& node);
    double g_,h_,f_, p_;
    int gi_,hi_,fi_,pi_;

    Point<int> pose_;
    Point<int> parent_;
    double f_inv_, eps_;
};

#endif // PLANNING_NODE_HPP
