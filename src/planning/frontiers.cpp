#include <planning/frontiers.hpp>
#include <planning/motion_planner.hpp>
#include <common/grid_utils.hpp>
#include <common/point.hpp>
#include <slam/occupancy_grid.hpp>
#include <lcmtypes/robot_path_t.hpp>
#include <queue>
#include <set>
#include <cassert>
#include <algorithm>


bool is_frontier_cell(int x, int y, const OccupancyGrid& map);
frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers);
robot_path_t path_to_frontier(const frontier_t& frontier,
                              const pose_xyt_t& pose,
                              const OccupancyGrid& map,
                              const MotionPlanner& planner);
pose_xyt_t nearest_navigable_cell(pose_xyt_t pose,
                                  Point<float> desiredPosition,
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner);
pose_xyt_t search_to_nearest_free_space(Point<float> position, const OccupancyGrid& map, const MotionPlanner& planner);
double path_length(const robot_path_t& path);


std::vector<frontier_t> find_map_frontiers(const OccupancyGrid& map,
                                           const pose_xyt_t& robotPose,
                                           double minFrontierLength)
{
    /*
    * To find frontiers, we use a connected components search in the occupancy grid. Each connected components consists
    * only of cells where is_frontier_cell returns true. We scan the grid until an unvisited frontier cell is
    * encountered, then we grow that frontier until all connected cells are found. We then continue scanning through the
    * grid. This algorithm can also perform very fast blob detection if you change is_frontier_cell to some other check
    * based on pixel color or another condition amongst pixels.
    */
    std::vector<frontier_t> frontiers;
    std::set<Point<int>> visitedCells;

    Point<int> robotCell = global_position_to_grid_cell(Point<float>(robotPose.x, robotPose.y), map);
    std::queue<Point<int>> cellQueue;
    cellQueue.push(robotCell);
    visitedCells.insert(robotCell);

    // Use a 4-way connected check for expanding through free space.
    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };

    // Do a simple BFS to find all connected free space cells and thus avoid unreachable frontiers
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);

            // If the cell has been visited or isn't in the map, then skip it
            if(visitedCells.find(neighbor) != visitedCells.end() || !map.isCellInGrid(neighbor.x, neighbor.y))
            {
                continue;
            }
            // If it is a frontier cell, then grow that frontier
            else if(is_frontier_cell(neighbor.x, neighbor.y, map))
            {
                frontier_t f = grow_frontier(neighbor, map, visitedCells);

                // If the frontier is large enough, then add it to the collection of map frontiers
                if(f.cells.size() * map.metersPerCell() >= minFrontierLength)
                {
                    frontiers.push_back(f);
                }
            }
            // If it is a free space cell, then keep growing the frontiers
            else if(map(neighbor.x, neighbor.y) < 0)
            {
                visitedCells.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }

    return frontiers;
}


robot_path_t plan_path_to_frontier(const std::vector<frontier_t>& frontiers,
                                   const pose_xyt_t& robotPose,
                                   const OccupancyGrid& map,
                                   const MotionPlanner& planner)
{
    ///////////// TODO: Implement your strategy to select the next frontier to explore here //////////////////
    /*
    * NOTES:
    *   - If there's multiple frontiers, you'll need to decide which to drive to.
    *   - A frontier is a collection of cells, you'll need to decide which one to attempt to drive to.
    *   - The cells along the frontier might not be in the configuration space of the robot, so you won't necessarily
    *       be able to drive straight to a frontier cell, but will need to drive somewhere close.
    */
    robot_path_t plannedPath;
    std::vector<double> frontier_scores;
    for (auto& frontier : frontiers) {
        frontier_scores.push_back(score_frontier(frontier, robotPose, map));
    }
    while (!frontier_scores.empty()){
        int best_frontier_idx = std::max_element(frontier_scores.begin(), frontier_scores.end()) - frontier_scores.begin();
        frontier_t best_frontier = frontiers[best_frontier_idx];
        while (!best_frontier.cells.empty()) {
            Point<float> desiredPosition = best_frontier.cells[0];
            best_frontier.cells.erase(best_frontier.cells.begin());
            pose_xyt_t goalPose = nearest_navigable_cell(robotPose, desiredPosition, map, planner);
            //goalPose.x = p.x; goalPose.y = p.y; goalPose.theta = 0.0f;
            plannedPath = planner.planPath(robotPose, goalPose);
            if (plannedPath.path.size() > 1) {
                //std::cout<<"Found a path! Executing now" << std::endl;
                return plannedPath;
            }
        }
        frontier_scores.erase(frontier_scores.begin() + best_frontier_idx);
        //frontiers.erase(frontiers.begin() + best_frontier_idx);
    }
    return plannedPath;
}

double score_frontier(const frontier_t& frontier, const pose_xyt_t& robotPose, const OccupancyGrid& map)
{
    double distanceWeight = 1.0, sizeWeight = 0.0;
    int frontierSize = frontier.cells.size();
    double dx = robotPose.x - frontier.cells[0].x;
    double dy = robotPose.y - frontier.cells[0].y;
    double distanceToFrontier = std::sqrt(dx*dx + dy*dy);
    return (distanceWeight*distanceToFrontier + sizeWeight*frontierSize);
}

pose_xyt_t nearest_navigable_cell(pose_xyt_t pose,
                                  Point<float> desiredPosition,
                                  const OccupancyGrid& map,
                                  const MotionPlanner& planner)
{
    pose_xyt_t navigablePose = {0};
    Point<int> desiredCell = global_position_to_grid_cell(desiredPosition, map);
    int x = desiredCell.x, y = desiredCell.y;
    //If desired position is already free, return the same.
    if(map.isCellInGrid(x,y) && map(x,y) < 0) {
        navigablePose.x = desiredPosition.x;
        navigablePose.y = desiredPosition.y;
        navigablePose.theta = 0.0;
        return navigablePose;
    }
    //Else conduct a search among the neighbouring cells.
    //const int kSafeCellRadius = std::lrint(std::ceil(0.5 * map.cellsPerMeter()));
    const int kSafeCellRadius = 4;
    const int kNumNeighbors = 8;
    const int xDeltas[] = {-1, -1, -1, 0, 0, 1, 1, 1};
    const int yDeltas[] = {-1, 0, 1, -1, 1, -1, 0, 1};
    int newX, newY;
    for (int n = 0; n < kNumNeighbors; ++n) {
        newX = x + xDeltas[n];
        newY = y + yDeltas[n];
        if (map.isCellInGrid(newX, newY) && map.logOdds(newX, newY) < 0) {
            break;
        }
    }
    const int xQuadrants[] = {-1, -1, 1, 1};
    const int yQuadrants[] = {-1, 1, 1, -1};
    for (int q = 0; q < 4; ++q) {
        int centerX = newX + (kSafeCellRadius-1) * xQuadrants[q];
        int centerY = newY + (kSafeCellRadius-1) * yQuadrants[q];
        Point<int> newpt(centerX, centerY);
        Point<float> centerP = grid_position_to_global_position(newpt, map);
        navigablePose.x = centerP.x;
        navigablePose.y = centerP.y;
        navigablePose.theta = 0.0;
        if (planner.isValidGoal(navigablePose)) {
          return navigablePose;
        }
    }
    return navigablePose;
}

bool is_frontier_cell(int x, int y, const OccupancyGrid& map)
{
    // A cell if a frontier if it has log-odds 0 and a neighbor has log-odds < 0

    // A cell must be in the grid and must have log-odds 0 to even be considered as a frontier
    if(!map.isCellInGrid(x, y) || (map(x, y) != 0))
    {
        return false;
    }

    const int kNumNeighbors = 4;
    const int xDeltas[] = { -1, 1, 0, 0 };
    const int yDeltas[] = { 0, 0, 1, -1 };

    for(int n = 0; n < kNumNeighbors; ++n)
    {
        // If any of the neighbors are free, then it's a frontier
        // Note that logOdds returns 0 for out-of-map cells, so no explicit check is needed.
        if(map.logOdds(x + xDeltas[n], y + yDeltas[n]) < 0)
        {
            return true;
        }
    }

    return false;
}


frontier_t grow_frontier(Point<int> cell, const OccupancyGrid& map, std::set<Point<int>>& visitedFrontiers)
{
    // Every cell in cellQueue is assumed to be in visitedFrontiers as well
    std::queue<Point<int>> cellQueue;
    cellQueue.push(cell);
    visitedFrontiers.insert(cell);

    // Use an 8-way connected search for growing a frontier
    const int kNumNeighbors = 8;
    const int xDeltas[] = { -1, -1, -1, 1, 1, 1, 0, 0 };
    const int yDeltas[] = {  0,  1, -1, 0, 1,-1, 1,-1 };

    frontier_t frontier;

    // Do a simple BFS to find all connected frontier cells to the starting cell
    while(!cellQueue.empty())
    {
        Point<int> nextCell = cellQueue.front();
        cellQueue.pop();

        // The frontier stores the global coordinate of the cells, so convert it first
        frontier.cells.push_back(grid_position_to_global_position(nextCell, map));

        // Check each neighbor to see if it is also a frontier
        for(int n = 0; n < kNumNeighbors; ++n)
        {
            Point<int> neighbor(nextCell.x + xDeltas[n], nextCell.y + yDeltas[n]);
            if((visitedFrontiers.find(neighbor) == visitedFrontiers.end())
                && (is_frontier_cell(neighbor.x, neighbor.y, map)))
            {
                visitedFrontiers.insert(neighbor);
                cellQueue.push(neighbor);
            }
        }
    }

    return frontier;
}