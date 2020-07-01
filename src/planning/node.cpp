#include <planning/node.hpp>
#include <algorithm>
#include <common/grid_utils.hpp>
#include <common/point.hpp>

Node::Node(void)
: pose_(0,0)
, parent_(0,0)
, g_(0.0)
, h_(0.0)
, f_(0.0)
, f_inv_(0.0)
, eps_(1E-6)
{
}

Node::Node(const pose_xyt_t& pose, const ObstacleDistanceGrid& grid)
: g_(0.0)
, h_(0.0)
, f_(0.0)
, p_(0.0)
, f_inv_(0.0)
, eps_(1E-6)
{
  Point<double> pose_point(pose.x, pose.y);
  Point<int> p = global_position_to_grid_cell(pose_point, grid);
  pose_.x = p.x; pose_.y = p.y;
  parent_.x = p.x; parent_.y = p.y;
}

// bool Node::operator == (const Node& node)
// {
//     if (node.pose_.x == pose_.x && node.pose_.y == pose_.y) {
//         return true;
//     }
//     return false;
// }

bool Node::operator != (const Node& node)
{
    if (node.pose_.x != pose_.x || node.pose_.y != pose_.y) {
        return true;
    }
    return false;
}

// bool Node::operator < (const Node& node)
// {
//     return (f_inv_ < node.f_inv_);
// }

bool Node::operator <= (const Node& node)
{
    return (f_inv_ <= node.f_inv_);
}

bool Node::operator > (const Node& node)
{
    return (f_inv_ > node.f_inv_);
}

bool Node::operator >= (const Node& node)
{
    return (f_inv_ >= node.f_inv_);
}

void Node::set_costs(double g, double h)
{
    g_ = g;
    h_ = h;
    f_ = g_ + h_ + p_;
    f_inv_ = 1.0 / (f_ + eps_);
}
