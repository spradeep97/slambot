#include <planning/astar.hpp>
#include <planning/obstacle_distance_grid.hpp>
#include <planning/node.cpp>
#include <cfloat>
#include <common/grid_utils.hpp>
#include <common/point.hpp>
#include <algorithm>


AstarSearch::AstarSearch(pose_xyt_t start,
                         pose_xyt_t goal,
                         const ObstacleDistanceGrid& distances,
                         const SearchParams& params)
: start_(start, distances)
, goal_(goal, distances)
{
    //print_node(goal_);
    openList_.insert(start_);
    //path_.path.push_back(start);
    distances_ = distances;
    params_ = params;
    foundDest_ = false;
    int height = distances_.heightInCells();
    int width = distances_.widthInCells();
    closedList_ = new bool* [height];
    nodeGrid_ = new Node* [height];
    for (int i = 0; i < height; ++i) {
        closedList_[i] = new bool[width];
        nodeGrid_[i] = new Node[width];
        for (int j = 0; j < width; ++j) {
            closedList_[i][j] = false;
            nodeGrid_[i][j].f_ = FLT_MAX;
            nodeGrid_[i][j].g_ = FLT_MAX;
            nodeGrid_[i][j].h_ = FLT_MAX;
            nodeGrid_[i][j].parent_.x = -1;
            nodeGrid_[i][j].parent_.y = -1;
            nodeGrid_[i][j].pose_.x = j;
            nodeGrid_[i][j].pose_.y = i;
            nodeGrid_[i][j].p_ = calc_proximity_cost(nodeGrid_[i][j]);
        }
    }

    //Initialize g and h costs for start node
    start_.set_costs(0.0, calc_h_cost(start_));
    int i = start_.pose_.x , j = start_.pose_.y;
    nodeGrid_[i][j].f_ = 0.0;
    nodeGrid_[i][j].g_ = 0.0;
    nodeGrid_[i][j].h_ = 0.0;
    nodeGrid_[i][j].p_ = calc_proximity_cost(nodeGrid_[i][j]);
    nodeGrid_[i][j].parent_.x = i;
    nodeGrid_[i][j].parent_.y = j;
    nodeGrid_[i][j].pose_.x = i;
    nodeGrid_[i][j].pose_.y = j;
}

AstarSearch::~AstarSearch()
{
    delete []closedList_;
    delete []nodeGrid_;
}

bool AstarSearch::isValid(Node& n)
{
    //return distances_.isCellInGrid(n.pose_.x, n.pose_.y);
    if(distances_.isCellInGrid(n.pose_.x, n.pose_.y))
    {
        // And is far enough from obstacles that the robot can physically occupy the space
        // Add an extra cell to account for discretization error and make motion a little safer by not trying to
        // completely snuggle up against the walls in the motion plan
        return distances_(n.pose_.x, n.pose_.y) > params_.minDistanceToObstacle;
    }
    return false;
}

bool AstarSearch::isFree(Node& n)
{
    return (std::abs(distances_(n.pose_.x, n.pose_.y)) > 1E-6);
}

bool AstarSearch::is_safe_cell(Node& n)
{
    int x = n.pose_.x, y = n.pose_.y;
    // Search a circular region around (x, y). If any of the cells within the robot radius are occupied, then the
    // cell isn't safe.
    const int kSafeCellRadius = std::lrint(std::ceil(params_.minDistanceToObstacle * distances_.cellsPerMeter()));

    for(int dy = -kSafeCellRadius; dy <= kSafeCellRadius; ++dy)
    {
        for(int dx = -kSafeCellRadius; dx <= kSafeCellRadius; ++dx)
        {
            // Ignore the corners of the square region, where outside the radius of the robot
            if(std::sqrt(dx*dx + dy*dy) * distances_.metersPerCell() > params_.minDistanceToObstacle)
            {
                continue;
            }

            // If the odds at the cells are greater than 0, then there's a collision, so the cell isn't safe
            if(distances_(x + dx, y + dy) == 0)
            {
                return false;
            }
        }
    }

    // The area around the cell is free of obstacles, so all is well
    return true;
}

bool AstarSearch::isGoal(Node& n)
{
    return (n == goal_);
}

void AstarSearch::makePath(Node& n)
{
    int row = n.pose_.y, col = n.pose_.x;
    pose_xyt_t p;
    Point<double> co_od;
    while (!(nodeGrid_[row][col] == start_)) {
        //print_node(nodeGrid_[row][col]);
        //Push back current node
        Node& curr = nodeGrid_[row][col];
        co_od = grid_position_to_global_position(curr.pose_, distances_);
        // p.x = (float)curr.pose_.x;
        // p.y = (float)curr.pose_.y;
        p.x = co_od.x;
        p.y = co_od.y;
        // float dy = curr.parent_.y - curr.pose_.y;
        // float dx = curr.parent_.x - curr.pose_.x;
        // p.theta = std::atan2(dy, dx);
        p.theta = 0.0f;
        path_.path.push_back(p);

        //Move to parent node
        int temp_row = nodeGrid_[row][col].parent_.y;
        int temp_col = nodeGrid_[row][col].parent_.x;
        row = temp_row;
        col = temp_col;
    }
    p.x = start_.pose_.x;
    p.y = start_.pose_.y;
    p.theta = 0;
    path_.path.push_back(p);
    std::reverse(path_.path.begin(), path_.path.end());
    //print_path();
}

double AstarSearch::calc_h_cost(Node& n)
{
    //L1 distance
    int dx = std::abs(n.pose_.x - goal_.pose_.x);
    int dy = std::abs(n.pose_.y - goal_.pose_.y);
    return (dx + dy) + (1.414 - 2)*std::min(dx,dy);
    //return (dx + dy);

    // //L2 distance
    // return ((double)std::sqrt((n.pose_.x - goal_.pose_.x)*(n.pose_.x - goal_.pose_.x) +
    //                           (n.pose_.y - goal_.pose_.y)*(n.pose_.y - goal_.pose_.y)));
}

double AstarSearch::calc_proximity_cost(Node& n)
{
    double cellDistance = distances_(n.pose_.x, n.pose_.y);
    if (isValid(n) && isFree(n)) {
        if (cellDistance > params_.maxDistanceWithCost) {
            return 0;
        } else {
            return (std::pow(params_.maxDistanceWithCost - cellDistance , params_.distanceCostExponent));
        }
    }
    return FLT_MAX;
}

// std::vector<Node> AstarSearch::expand(Node& n)
// {
//     std::vector<Node> kids;
//     int col = n.pose_.x , row = n.pose_.y;
//     for (int dy = -1; dy <= 1; ++dy) {
//         for (int dx = -1; dx <= 1; ++dx) {
//             if(distances_.isCellInGrid(row+dy, col+dx)) {
//                 if (isValid(nodeGrid_[row + dy][col + dx]) && isFree(nodeGrid_[row + dy][col + dx])
//                     && is_safe_cell(nodeGrid_[row + dy][col + dx])) {
//                     kids.push_back(nodeGrid_[row + dy][col + dx]);
//                 }
//             }
//         }
//     }
//     return kids;
// }

void AstarSearch::updateKid(Node& kid, Node& parent)
{
    int col = kid.pose_.x, row = kid.pose_.y;
    int col_p = parent.pose_.x, row_p = parent.pose_.y;

    //If this kid is the goal, set its parent and set foundDest_ flag
    if (isGoal(kid)) {
        //std::cout << "Goal found" << std::endl;
        nodeGrid_[row][col].parent_.x = parent.pose_.x;
        nodeGrid_[row][col].parent_.y = parent.pose_.y;
        //print_node(nodeGrid_[row][col]);
        makePath(nodeGrid_[row][col]);
        foundDest_ = true;
        //1.std::cout << "Found!" << std::endl;
        return;
    }

    //Update the kid's node values only if its not closed and its free
    else if (closedList_[row][col] == false &&
               isFree(kid)) {
        double gNew;
        if (std::abs(parent.pose_.x - kid.pose_.x) == 1 &&
            std::abs(parent.pose_.y - kid.pose_.y) == 1) {
                gNew = parent.g_ + 1.4;
        } else {
                gNew = parent.g_ + 1.0;
        }
        double old_h = nodeGrid_[row][col].h_;
        //double gNew = parent.g_ + 1.0;
        double hNew = calc_h_cost(kid);
        double fNew = gNew + hNew;

        //Update the kid's costs only if the new costs are cheaper
        if (nodeGrid_[row][col].f_ == FLT_MAX || nodeGrid_[row][col].f_ > fNew) {
            // kid.set_costs(gNew, hNew);
            // kid.parent_.x = parent.pose_.x;
            // kid.parent_.y = parent.pose_.y;
            nodeGrid_[row][col].set_costs(gNew, hNew);
            nodeGrid_[row][col].parent_.x = parent.pose_.x;
            nodeGrid_[row][col].parent_.y = parent.pose_.y;
            openList_.insert(nodeGrid_[row][col]);
        }
    }
}

void AstarSearch::searchPath()
{
    if (!isValid(start_)) {
        std::cout << "Invalid start node." << std::endl;
        return;
    }
    if (!isValid(goal_)) {
        std::cout << "Invalid goal node." << std::endl;
        return;
    }
    if (!(isFree(start_) && isFree(goal_))) {
        std::cout << "Either start node or goal node is an obstacle." << std::endl;
        return;
    }
    if (isGoal(start_)) {
        std::cout << "We are already at the goal." << std::endl;
        return;
    }
    int col, row;
    while (!openList_.empty()) {
        //std::cout << "Entered while loop." << std::endl;
        Node n = *openList_.begin();
        //print_node(n);
        openList_.erase(openList_.begin());
        col = n.pose_.x;
        row = n.pose_.y;
        closedList_[row][col] = true;
        std::vector<Node> kids = expand(n);
        for (auto& kid : kids) {
            //std::cout << "Kid before->" << std::endl;
            //print_node(kid);
            updateKid(kid, n);
            if(foundDest_) {
                return;
            }
            //print_node(kid);
            //std::cout << "<-Kid after\n-----" << std::endl;
        }
    }
    if (foundDest_ == false) {
        std::cout << "Could not find a valid path from start to goal." << std::endl;
    }
    return;
}

void AstarSearch::print_closedList(void)
{
    for (int i = 0; i < distances_.heightInCells(); ++i) {
        for (int j = 0; j < distances_.widthInCells(); ++j) {
            std::cout << closedList_[i][j] << " ";
        }
        std::cout << "\n";
    }
}

void AstarSearch::print_node(Node& n)
{
    Point<double> p = grid_position_to_global_position(n.pose_, distances_);
    std::cout << "Node:\npose:" << n.pose_.x << "|" << n.pose_.y  << "||" << p.x << "|" << p.y << std::endl;
    std::cout << "parent:" << n.parent_.x << "|" << n.parent_.y << std::endl;
    std::cout << "g:" << n.g_ << " | h:" << n.h_ << " | f:" << n.f_ << " | p:" << n.p_ << "\n-----" << std::endl;
}

void AstarSearch::print_path(void)
{
    for (auto pose : path_.path) {
        std::cout << pose.x << "|" << pose.y << "|" << pose.theta << std::endl;
    }
}

robot_path_t search_for_path(pose_xyt_t start,
                             pose_xyt_t goal,
                             const ObstacleDistanceGrid& distances,
                             const SearchParams& params)
{
    ////////////////// TODO: Implement your A* search here //////////////////////////

    robot_path_t path;
    path.utime = start.utime;
    // path.path.push_back(start);
    AstarSearch astar(start, goal, distances, params);
    astar.searchPath();
    path = astar.path_;
    path.path_length = path.path.size();

    return path;
}
